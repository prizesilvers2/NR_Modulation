
#define _GNU_SOURCE
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <math.h>
#ifndef BUILDOPT_MACOS
#include <malloc.h>
#endif

#include "vThread_RF.h"
#include "NPAL_thread.h"
#include "NPAL_engines.h"
#include "NPAL_time.h"
#include "../../ARCH/COMMON/common_lib.h"
#include "vThreadix_rf.h"
#include "vThread_HWCommon.h"



//#define VRF_DBGPROC



#define VRF_FREQOFFSET_STEPSIZE		100

#define VRF_TIMEDRIFT_DELTA			2


//TX related parameters
#ifdef MOD_STBC
#define VRF_IQSENDMARGIN			250 //us	//change STBC : 300, STLC : 250
#else
#define VRF_IQSENDMARGIN			250 //us
#endif
#define VRF_MINSCHEDTIME			100  //us


/* virtual H/W for RF 
- Assuming that there is only one RF in the UE
- virtual H/W has one real RF device and has HAL code for controling this real HW
*/


/*     ------- RF buffer ------          */
// TX/RX chain vHW interface
int32_t __attribute__ ((aligned (32))) vrf_rxdata[VHW_NB_SAMPLEBANK][HW_NB_RXANT][VHW_NB_MAXSAMPLES];
int32_t __attribute__ ((aligned (32))) vrf_txdata[VHW_NB_SAMPLEBANK][HW_NB_TXANT][VHW_NB_MAXSAMPLES];
int32_t __attribute__ ((aligned (32))) vrf_silentdata[HW_NB_TXANT][VHW_NB_MAXSAMPLES];


//frame parameters
static uint32_t vrf_samples_in_slot;
static int __attribute__ ((aligned (32))) dummy_rx[HW_NB_RXANT][VHW_NB_MAXSAMPLES];

/*     ------- RF HW related ------          */

//1. SDR RF interface
static openair0_device vrf_rfdevice; 	//rf device object
static openair0_config_t vrf_rfDevcfg[MAX_CARDS];
static uint8_t vrf_rfHwOn; 				//RF device on/off status
static double vrf_rxcFreq[4];			//current RX center frequency
static double vrf_txcFreq[4];			//current TX center frequency

//2. initial configuration parameters for SDR RF 
static unsigned int vrf_mmapped_dma;
static clock_source_t vrf_clksrc = internal;
static char vrf_config_file[1024];
static char* vrf_usrpArg = NULL;


#ifdef DUMPRFTX
static int printCnt=0;
#endif

/*     ------- HAL - HW interface related ------          */
const halRfic_woReg_t* vrf_HALix_in; 	//write only registers
halRfic_roReg_t* vrf_HALix_out;			//read only registers
halRfic_rwReg_t* vrf_HALix_buf;			//read/write registers
pthread_mutex_t* vrf_muPtr_rfHal;		//MUTEX for HAL interface
pthread_cond_t* vrf_csPtr_rfHal;		//condition signal for off -> on

static struct {
	int freq_offset; //delta
	int sync_offset;
	int rx_offset;
	rfMode_e mode;
} vrf_rtCfg; //RFIC HW abstraction

#ifdef MOD_STLC
static struct {
	//feedback channel
	uint16_t slotBmp;
	uint32_t offset;
	uint32_t duration;
} vrf_fdbkConfig;
#endif
static uint32_t vrf_IQSendMargin;
static uint8_t vrf_slotDelay=0;
static uint32_t vrf_SchedTimeSample=0; //in sample
static uint32_t vrf_SchedTime=0; //in us
static uint8_t vrf_activatedAnt = 0;


/* ---------- HW - HW interfaces -----------       */

/* 1. RF - SRCH */
static vrfix_Reg_t					vrf_rfIxReg;



/* -------- 1. Initializing function codes ----------- */


//interface configuration between RF and TX/RX
vrfix_Reg_t* vrf_configRfReg(void)
{
	return &vrf_rfIxReg;
}



//initiation function for virtual RF H/W called by API
void vrf_initHwParams(unsigned int Vmmapped_dmam, int VclkSrc, char* inConfigFile, char* inUsrpArg)
{
	vrf_mmapped_dma = Vmmapped_dmam;

	vrf_clksrc = (clock_source_t)VclkSrc;

	if (inConfigFile != NULL)
		memcpy(vrf_config_file, inConfigFile, 1024*sizeof(char));
	if (inUsrpArg != NULL)
	{
		vrf_usrpArg = malloc((strlen(inUsrpArg)+1)*sizeof(char));
		strcpy(vrf_usrpArg, inUsrpArg);
	}
	
}

//initializing the abstraction variable for USRP
//called within this vHW
static void vrf_init_rfDevCfg(void) {

    int card;
    int i;

    for (card=0; card<MAX_CARDS; card++)
	{
        vrf_rfDevcfg[card].mmapped_dma = vrf_mmapped_dma;
        vrf_rfDevcfg[card].configFilename = NULL;
  		vrf_rfDevcfg[card].duplex_mode = duplex_mode_FDD;
		vrf_rfDevcfg[card].Mod_id = 0;
	
		vrf_rfDevcfg[card].clock_source = vrf_clksrc;
		vrf_rfDevcfg[card].digital_freq_offset = 0;
	
		vrf_rfDevcfg[card].tx_num_channels = min(2,HW_NB_TXANT);
		vrf_rfDevcfg[card].rx_num_channels = min(2,HW_NB_RXANT);
	
		for (i=0; i<4; i++)
		{
    		vrf_rfDevcfg[card].tx_freq[i]=0.0;
    		vrf_rfDevcfg[card].rx_freq[i]=0.0;	  
	  		vrf_rfDevcfg[card].autocal[i] = 1;
	  		vrf_rfDevcfg[card].tx_gain[i] = 0.0;
	  		vrf_rfDevcfg[card].rx_gain[i] = 0.0;
		}
		
		vrf_rfDevcfg[card].configFilename = vrf_config_file;
		if (vrf_usrpArg)
		{
			vrf_rfDevcfg[card].sdr_addrs = vrf_usrpArg;
		}
		vrf_rfDevcfg[card].antMode = antMode_trxSeparated;
    }
}

//configuration function for off-on procedures
static void vrf_configInit(void)
{
	//check other configurations
	int i;	

	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");

	vrf_rtCfg.mode = vrf_HALix_in->mode;	
	switch(vrf_HALix_in->samplingMode)
	{
		case VRF_SAMPLEMODE_192:
			vrf_rfDevcfg[0].sample_rate = 1.92e6;
			break;
		case VRF_SAMPLEMODE_768:
			vrf_rfDevcfg[0].sample_rate = 7.68e6;
			break;
		case VRF_SAMPLEMODE_1536:
			vrf_rfDevcfg[0].sample_rate = 15.36e6;
			break;
		case VRF_SAMPLEMODE_3072:
			vrf_rfDevcfg[0].sample_rate = 30.72e6;
			break;
		default:
			printf("[HW RF] WARNING : unknown sample rate %i (default 30.72e6 is set)\n", vrf_HALix_in->samplingMode);
			vrf_rfDevcfg[0].sample_rate = 30.72e6;
			break;
	}

	//other frame parameter definition
	vrf_samples_in_slot = vrf_HALix_in->samples_per_slot;

	vrf_rfDevcfg[0].tx_num_channels = vrf_HALix_in->nbActAnt;
	vrf_rfDevcfg[0].rx_num_channels = vrf_HALix_in->nbActAnt;
	vrf_activatedAnt = vrf_HALix_in->nbActAnt;

	LOG_E(PHY, "Activated RF channel (TX/RX) : %i / %i\n", vrf_rfDevcfg[0].tx_num_channels, vrf_rfDevcfg[0].rx_num_channels);
	for (i=0;i<HW_NB_TXANT;i++)
	{
		vrf_rfDevcfg[0].tx_freq[i] = vrf_HALix_in->tx_freq[i];
		vrf_txcFreq[i] = vrf_HALix_in->tx_freq[i];
	}
	for (i=0;i<HW_NB_RXANT;i++)
	{
		vrf_rfDevcfg[0].rx_freq[i] = vrf_HALix_in->rx_freq[i];
		vrf_rxcFreq[i] = vrf_HALix_in->rx_freq[i];
	}

	vrf_rfDevcfg[0].rx_bw = vrf_HALix_in->rx_bw;
	vrf_rfDevcfg[0].tx_bw = vrf_HALix_in->tx_bw;

	for (i=0;i<HW_NB_RXANT;i++)
	{
		vrf_rfDevcfg[0].rx_gain[i] = vrf_HALix_in->rx_gain[i];
		vrf_rfDevcfg[0].rx_gain_offset[i] = vrf_HALix_in->rx_gain_offset[i];
	}
	for (i=0;i<HW_NB_TXANT;i++)
	{
		vrf_rfDevcfg[0].tx_gain[i] = vrf_HALix_in->tx_gain[i];
	}
	vrf_HALix_buf->gainChanged = 0;

	vrf_IQSendMargin = (uint32_t)(VRF_IQSENDMARGIN*(vrf_rfDevcfg[0].sample_rate/1000000.0));
	vrf_SchedTimeSample = (uint32_t)(VRF_MINSCHEDTIME*(vrf_rfDevcfg[0].sample_rate/1000000.0));
	vrf_SchedTime = VRF_MINSCHEDTIME;
#ifdef MOD_STLC
	if (vrf_HALix_in->fdbkSlot != 0)
	{
		vrf_fdbkConfig.slotBmp = vrf_HALix_in->fdbkSlot;
		vrf_fdbkConfig.offset = vrf_HALix_in->fdbkOffset;
		vrf_fdbkConfig.duration = vrf_HALix_in->fdbkDuration;

		if (vrf_samples_in_slot - (vrf_fdbkConfig.offset + vrf_fdbkConfig.duration) < vrf_IQSendMargin)
		{
			vrf_IQSendMargin = vrf_samples_in_slot - (vrf_fdbkConfig.offset + vrf_fdbkConfig.duration) - vrf_SchedTimeSample;
			vrf_slotDelay = 1;
		}
		else
		{
			vrf_SchedTimeSample = vrf_samples_in_slot - (vrf_fdbkConfig.offset + vrf_fdbkConfig.duration) - vrf_IQSendMargin;
			vrf_SchedTime = (uint32_t)(vrf_SchedTimeSample*(1000000.0/vrf_rfDevcfg[0].sample_rate));
		}

		LOG_E(PHY, "feedback configuration ::: slotbmp : %x, offset : %i, duration : %i, sched margin : %i, slot sample : %i, vrf_slotDelay : %i, schedTimeSample : %i, schedTime : %i, fdbk gap : %i\n",
			vrf_fdbkConfig.slotBmp, vrf_fdbkConfig.offset, vrf_fdbkConfig.duration, vrf_IQSendMargin, vrf_samples_in_slot, vrf_slotDelay, vrf_SchedTimeSample, vrf_SchedTime, (uint32_t)((vrf_samples_in_slot - (vrf_fdbkConfig.offset + vrf_fdbkConfig.duration))*(1000000.0/vrf_rfDevcfg[0].sample_rate)));
	}
	else
	{
		vrf_fdbkConfig.slotBmp = 0;
	}
#endif
	vrf_rfDevcfg[0].antMode = antMode_trxSeparated;
	if (vrf_HALix_in->antMode == 1)
		vrf_rfDevcfg[0].antMode = antMode_trxCommon;
	

	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	
	vrf_rtCfg.freq_offset = 0;
	vrf_rtCfg.sync_offset = 0;
	vrf_rtCfg.rx_offset = 0;
	vrf_rfDevcfg[0].digital_freq_offset = 0;
	vrf_HALix_out->digital_freq_offset = 0;
}









/* -------- 2. real-time processing codes ----------- */
//main function for processing the real-time register command of this virtual RF
//inner function
static uint8_t vrf_configRealTime(void)
{
	uint8_t flag_cfg = 0;
	int i;

	//reading register area and get values if changed
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	// -- Frequency offset (AFC)
	if (vrf_rtCfg.freq_offset != vrf_HALix_in->freq_offset)
	{
		vrf_rtCfg.freq_offset = vrf_HALix_in->freq_offset;
		flag_cfg |= 0x01;
	}
	
	// -- RX offsets
	//jumping offset
	if (vrf_HALix_buf->sync_offset != 0)
	{
		vrf_rtCfg.sync_offset = vrf_HALix_buf->sync_offset;
		vrf_HALix_buf->sync_offset = 0;
	}
	//real time drift offset
	if (vrf_HALix_in->rx_offset != vrf_rtCfg.rx_offset)
	{
		vrf_rtCfg.rx_offset = vrf_HALix_in->rx_offset;
	}
	
	// -- Gains
	if (vrf_HALix_buf->gainChanged == 1)
	{
		for (i=0;i<HW_NB_RXANT;i++)
		{
			vrf_rfDevcfg[0].rx_gain[i] = vrf_HALix_in->rx_gain[i];
			vrf_rfDevcfg[0].rx_gain_offset[i] = vrf_HALix_in->rx_gain_offset[i];
		}
		for (i=0;i<HW_NB_TXANT;i++)
		{
			vrf_rfDevcfg[0].tx_gain[i] = vrf_HALix_in->tx_gain[i];
		}
		vrf_HALix_buf->gainChanged = 0;
		flag_cfg |= 0x02;
	}
  	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	
	//gain setting
	if (flag_cfg & 0x02)
	{
		vrf_rfdevice.trx_set_gains_func(&vrf_rfdevice, vrf_rfDevcfg);
	}
	
	return flag_cfg;
}

//reading on/off register in real-time
//inner function
static uint8_t vrf_readOnOff(void)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading onOff");
	onOff = vrf_HALix_in->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading onOff");

	return onOff;
}




/* --------- 3. USRP trx function (all inner functions) -------- */
//throw away samples of one frame
void vrf_bypassSlot(openair0_timestamp *timestamp)
{
	void *rxp[HW_NB_RXANT];

	for (int i=0; i<vrf_activatedAnt; i++)
		rxp[i] = ((void *)&dummy_rx[i][0]);
	

	AssertFatal(vrf_samples_in_slot ==                         // Add RXANT_ID
				vrf_rfdevice.trx_read_func(&vrf_rfdevice,
				                           timestamp,
				                           rxp,
				                           vrf_samples_in_slot,
				                           vrf_activatedAnt),
				                           "");
}

#ifdef MOD_STLC
void vrf_readFdbk(openair0_timestamp *timestamp, uint8_t slot_nb)
{
	void *rxp[HW_NB_RXANT];

	
	// 1. bypass
	for (int i=0; i<vrf_activatedAnt; i++)
	{
		memset(&vrf_rxdata[slot_nb][i][0], 0, vrf_fdbkConfig.offset*sizeof(int)); //clear the sample part at the beginning
		rxp[i] = ((void *)&dummy_rx[i][0]);
	}
		

	AssertFatal(vrf_fdbkConfig.offset ==                        
				vrf_rfdevice.trx_read_func(&vrf_rfdevice,
				                           timestamp,
				                           rxp,
				                           vrf_fdbkConfig.offset,
				                           vrf_activatedAnt),
				                           "");


	//2. read fdbk
	for (int i=0; i<vrf_activatedAnt; i++)
		rxp[i] = ((void *)&vrf_rxdata[slot_nb][i][vrf_fdbkConfig.offset]);

	AssertFatal(vrf_fdbkConfig.duration ==                      // Add RXANT_ID
				 vrf_rfdevice.trx_read_func(&vrf_rfdevice,
	                                        timestamp,
	                                        rxp,
	                                        vrf_fdbkConfig.duration,
	                                        vrf_activatedAnt), 
											"");
}
#endif


//throw away samples till the next frame boundary
void vrf_syncInSlot(openair0_timestamp *timestamp, int sync_offset)
{
	void *rxp[HW_NB_RXANT];
	
	for (int i=0; i<vrf_activatedAnt; i++)
		rxp[i] = ((void *)&vrf_rxdata[0][i][0]);

	LOG_E(PHY, "[vRF] Resynchronizing RX by %d samples\n", sync_offset);

	while (sync_offset < 0)
	{
		sync_offset += vrf_samples_in_slot;
	}
	
	for ( int size = sync_offset ; size > 0 ; size -= vrf_samples_in_slot )
	{
  		int unitTransfer = size > vrf_samples_in_slot ? vrf_samples_in_slot : size ;
  		AssertFatal(unitTransfer ==                                         // Add RXANT_ID
					vrf_rfdevice.trx_read_func(&vrf_rfdevice,
		                                         timestamp,
		                                         rxp,
		                                         unitTransfer,
		                                         vrf_activatedAnt),
												 "");
	}
}


void vrf_sendSlot(openair0_timestamp *timestamp, uint8_t slot_nr, uint8_t slot_offset, int txrxOffset)
{
	void *txp[HW_NB_TXANT];
	int writeBlockSize = vrf_samples_in_slot;

	for (int i=0; i<vrf_activatedAnt; i++)
		txp[i] = (void *)&vrf_txdata[slot_nr][i][0];

	AssertFatal( writeBlockSize ==                                        // Add RXANT_ID
		   		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
							                   *timestamp  + txrxOffset + 
							                   slot_offset*vrf_samples_in_slot,
							                   txp,
							                   writeBlockSize,
							                   vrf_activatedAnt,
							                   4),"");
}

#ifdef MOD_STLC
void vrf_sendFdbk(openair0_timestamp *timestamp, uint8_t slot_nr)
{
	void *txp[HW_NB_TXANT];
	int writeBlockSize = vrf_fdbkConfig.duration;

	for (int i=0; i<vrf_activatedAnt; i++)
		txp[i] = (void *)&vrf_txdata[slot_nr][i][vrf_fdbkConfig.offset];

	AssertFatal( writeBlockSize ==                                        // Add RXANT_ID
		   		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
							                   *timestamp + vrf_fdbkConfig.offset +
							                   2*vrf_samples_in_slot,
							                   txp,
							                   writeBlockSize,
							                   vrf_activatedAnt,
							                   4),"");
}

void vrf_sendSlotBySample(openair0_timestamp *timestamp, uint8_t slot_nr, uint32_t sample_offset, int txrxOffset)
{
	void *txp[HW_NB_TXANT];
	int writeBlockSize = vrf_samples_in_slot;

	for (int i=0; i<vrf_activatedAnt; i++)
		txp[i] = (void *)&vrf_txdata[slot_nr][i][0];

	AssertFatal( writeBlockSize ==                                        // Add RXANT_ID
		   		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
							                   *timestamp + txrxOffset +
							                   sample_offset,
							                   txp,
							                   writeBlockSize,
							                   vrf_activatedAnt,
							                   1),"");
}
#endif


void vrf_sendSilentSlot(openair0_timestamp *timestamp, uint8_t slot_offset, int txrxOffset)
{
	void *txp[HW_NB_TXANT];
	int writeBlockSize = vrf_samples_in_slot;

	for (int i=0; i<vrf_activatedAnt; i++)
		txp[i] = (void *)&vrf_silentdata[i][0];

	AssertFatal( writeBlockSize ==                                        // Add RXANT_ID
		   		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
							                   *timestamp + txrxOffset +
							                   slot_offset*vrf_samples_in_slot,
							                   txp,
							                   writeBlockSize,
							                   vrf_activatedAnt,
							                   1),"");
}


//read one slot samples
void vrf_readSlot(openair0_timestamp *timestamp, uint8_t slot_nb, int deltaDrift)
{
	void *rxp[HW_NB_RXANT];

	for (int i=0; i<vrf_activatedAnt; i++)
		rxp[i] = ((void *)&vrf_rxdata[slot_nb][i][0]);

	AssertFatal((vrf_samples_in_slot+deltaDrift )==                      // Add RXANT_ID
				 vrf_rfdevice.trx_read_func(&vrf_rfdevice,
	                                        timestamp,
	                                        rxp,
	                                        vrf_samples_in_slot+deltaDrift,
	                                        vrf_activatedAnt), 
											"");

}


//read one slot samples
void vrf_bypassSample(openair0_timestamp *timestamp, int nbSample)
{
	void *rxp[HW_NB_RXANT];

	for (int i=0; i<vrf_activatedAnt; i++)
		rxp[i] = ((void *)&dummy_rx[i][0]);

	AssertFatal(nbSample ==                         // Add RXANT_ID
				vrf_rfdevice.trx_read_func(&vrf_rfdevice,
				                           timestamp,
				                           rxp,
				                           nbSample,
				                           vrf_activatedAnt),
				                           "");
}



int vrf_checkRxStatus(int processId)
{
	int onoffStatus;
	int processStatus;

	AssertFatal( 0 == pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
	onoffStatus = (vrf_rfIxReg.sharedReg.processBitmap & VRF_PROCBITMAP_MASK_ONOFF);
	processStatus = (vrf_rfIxReg.sharedReg.processBitmap & (1<<processId));
	AssertFatal( 0 == pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

	if (onoffStatus == 0)
	{
		return 0;
	}
	
	return (processStatus);
}


void vrf_irqRx(int processId)
{
	AssertFatal( 0 == pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
	vrf_rfIxReg.sharedReg.processBitmap |= (1<<processId);
	AssertFatal( 0 == pthread_cond_signal(&vrf_rfIxReg.sharedReg.irq_txrxInst), "");
	AssertFatal( 0 == pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
}



void vrf_initHALInterface(void* arg)
{
	vrf_HALix_in = &( ((ix_halRfic_t*)arg)->woReg  );
	vrf_HALix_out = &( ((ix_halRfic_t*)arg)->roReg );
	vrf_HALix_buf = &( ((ix_halRfic_t*)arg)->rwReg );
	vrf_muPtr_rfHal = (pthread_mutex_t*)&(((ix_halRfic_t*)arg)->mutex_rfHal);
	vrf_csPtr_rfHal = (pthread_cond_t*)&(((ix_halRfic_t*)arg)->cond_rfHal);

	vrf_HALix_out->hwStatus = rfst_null;
	vrf_HALix_out->ptr_rxdata = (const int32_t***) vrf_rxdata;
	vrf_HALix_out->ptr_txdata = (const int32_t***) vrf_txdata;
}


void vrf_initRfInterface(void)
{
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		
		for (int j=0;j<HW_NB_RXANT;j++)
		{
			vrf_rfIxReg.roReg.rxData[i][j] = vrf_rxdata[i][j];
		}
	}
	
	vrf_rfIxReg.roReg.hwStatus = hwIx_null;
	pthread_mutex_init(&(vrf_rfIxReg.roReg.regMutex), NULL);

	vrf_rfIxReg.sharedReg.syncOffset = 0;
	vrf_rfIxReg.sharedReg.freqDrift = 0;
	vrf_rfIxReg.sharedReg.timeDrift = 0;
#ifdef MOD_STLC
	vrf_rfIxReg.sharedReg.slotOffset = 0;
	vrf_rfIxReg.sharedReg.syncStatus = 0;
#endif
	vrf_rfIxReg.sharedReg.processBitmap= 0;
	//vrf_rfIxReg.sharedReg.txData = vrf_txdata;
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		
		for (int j=0;j<HW_NB_TXANT;j++)
		{
			vrf_rfIxReg.sharedReg.txData[i][j] = vrf_txdata[i][j];
		}
	}
	
	pthread_mutex_init(&(vrf_rfIxReg.sharedReg.sharedMutex), NULL);
	pthread_cond_init(&(vrf_rfIxReg.sharedReg.irq_txrxInst), NULL);

}


static void vrf_changeHwStats(rfHwStatus_e status)
{
	hwIxStatus_e hwIxStatus;

	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
	vrf_HALix_out->hwStatus = status;
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");
	

	if (status == rfst_off)
	{
		hwIxStatus = hwIx_off;
	}
	else if (status == rfst_on)
	{
		hwIxStatus = hwIx_on;
	}
	else
	{
		hwIxStatus = hwIx_null;
	}
	AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.roReg.regMutex)), "");
	vrf_rfIxReg.roReg.hwStatus = hwIxStatus;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.roReg.regMutex)), "");
}




//calculating the frequency offset input (into USRP)
//inner function 
static int vrf_calc_freqOffsetIn(int freq_offset)
{
	int truncFreq = 0;
	
	//drift command
	if (freq_offset > VRF_FREQOFFSET_STEPSIZE)
		truncFreq = VRF_FREQOFFSET_STEPSIZE;
	else if (freq_offset < -VRF_FREQOFFSET_STEPSIZE)
		truncFreq = -VRF_FREQOFFSET_STEPSIZE;
	
	return truncFreq;
}



static uint8_t vrf_check_processBitmap(uint8_t procCnt)
{
	uint8_t res;
	AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
	res = (vrf_rfIxReg.sharedReg.processBitmap & (1<<procCnt));
	AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

	return res;
}

static void vrf_clear_processBitmap(uint8_t procCnt)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
	vrf_rfIxReg.sharedReg.processBitmap &= (~(1<<procCnt));
	AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
}


static void vrf_set_processBitmap(uint8_t procCnt)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
	vrf_rfIxReg.sharedReg.processBitmap |= (1<<procCnt);
	AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
}


void vrf_irqTx(uint8_t slot_nr)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.roReg.regMutex)), "");
	vrf_rfIxReg.roReg.slot_nr = slot_nr;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.roReg.regMutex)), "");
	vrf_clear_processBitmap((slot_nr+vrf_slotDelay)%VHW_NB_SAMPLEBANK);

	AssertFatal( 0 == pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
	AssertFatal( 0 == pthread_cond_signal(&vrf_rfIxReg.sharedReg.irq_txrxInst), "");
	AssertFatal( 0 == pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
}



void vrf_pushBuffer(openair0_timestamp *timestamp, int offset, uint8_t slot)
{
	//check processBitmap and send TX buffer
	if (vrf_check_processBitmap(slot) == 0)
	{
		LOG_E(PHY, "[WARNING] sending data not prepared in VTX buffer, so sending cleared symbols in slot %i\n", slot);
		vrf_set_processBitmap(slot);
		vrf_sendSilentSlot(timestamp, vrf_slotDelay, offset);
	}
	else
	{
		vrf_sendSlot(timestamp, slot, vrf_slotDelay, offset);
	}
}

void *vRFIC_mainThread(void *arg)
{
	uint8_t onOffCfg = 0;
	openair0_timestamp timestamp;
	int i;
	char threadname[128];
	int slot_nr=0, sched_slot_nr=0;
#ifdef VRF_DBGPROC
	long long cur_time, next_time;
#endif

	vrf_rfIxReg.roReg.hwStatus = hwIx_null;

	//HW interface configuration
	// --------------- thread setting -------------------
	sprintf(threadname, "vRF");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY, threadname);
	
	// -------------- HAL and HW interface configuration -------------------
	vrf_initHALInterface(arg);
	vrf_initRfInterface();
	for (i=0;i<HW_NB_TXANT;i++)
		memset(&vrf_silentdata[i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int));
		
	//SDR RF initialization
	vrf_init_rfDevCfg();
	vrf_configInit();

	AssertFatal(0== openair0_device_load(&vrf_rfdevice, &vrf_rfDevcfg[0]), "");	
	vrf_rfdevice.host_type = RAU_HOST;
	
	


	vrf_changeHwStats(rfst_off);
	printf("[vHW] RF virtual HW is initialized, waiting for on signal\n");

	//outer loop for off waiting
	while (!NPAL_checkEnd())
	{
		//off state loop (stay until on signal comes)
		AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
		while (vrf_HALix_in->onoff == 0)
		      pthread_cond_wait( vrf_csPtr_rfHal, vrf_muPtr_rfHal );
		AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");

		
		//check on/off register and process it
		//if register is 'on', then do static configuration and apply to the SDR RF device (init and start)
		if (vrf_rfHwOn == 0)
		{
			vhw_sysTimeStamp_t onTimer;
			vhw_setTimeStamp(&onTimer);
			
		  	vrf_configInit();			
			AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.roReg.regMutex)), "");
			vrf_rfIxReg.roReg.slotSize = vrf_samples_in_slot;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.roReg.regMutex)), "");
			
			vrf_rfdevice.trx_configure_on(&vrf_rfdevice, vrf_rfDevcfg);
			AssertFatal(vrf_rfdevice.trx_start_func(&vrf_rfdevice) == 0, "Could not start the RF device\n");

			vrf_bypassSlot(&timestamp); //initial reception for setting the timestamp correctly

			vrf_changeHwStats(rfst_on);
			vrf_rfHwOn = 1;

			printf("[vHW] >>>>>>>>>>>>>>>>> RF virtual HW is on! (carrier DL freq : %lf, mode:%i) - on time : %4d us\n", 
				vrf_rfDevcfg[0].rx_freq[0], vrf_rtCfg.mode, vhw_getTime_us(&onTimer));
			#ifdef VRF_DBGPROC
			cur_time = npal_rdtsc();
			#endif

		}		
		
		//inner loop for on operation
		while ( !NPAL_checkEnd() && 
				 vrf_rfHwOn == 1 && 
				 (onOffCfg = vrf_readOnOff()) == 1 )
		{		
			//read the real-time configure register and apply in real-time
			//uint8_t flag_rtcfg = vrf_configRealTime();
			vrf_configRealTime();
			int deltaDrift = 0;

			switch (vrf_rtCfg.mode)
			{
				case rfmode_rx:

					//slot operation (TX or RX)
					#ifdef MOD_STLC
					AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
					int syncStatus = vrf_rfIxReg.sharedReg.syncStatus; 
					AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

					if(syncStatus == 1 && vrf_fdbkConfig.slotBmp & (0x01<<slot_nr))  //UL sendslot (UE to BS : pilot transmission) 
					{
						LOG_D(PHY, "Uplink in slot : %i\n", slot_nr);

						vrf_bypassSample(&timestamp, vrf_samples_in_slot-vrf_IQSendMargin-vrf_SchedTimeSample);  //Guard Band

						vrf_irqRx(slot_nr);
						usleep(vrf_SchedTime);

						//sending fdbk slot (preamble & pilot)
						vrf_sendFdbk(&timestamp, slot_nr);
						//bypass the rest
						vrf_bypassSample(&timestamp, vrf_IQSendMargin+vrf_SchedTimeSample);
					}
					else  //normal DL readslot (BS to UE : slot decoding)
					#endif
					{
						LOG_D(PHY, "Downlink in slot : %i\n", slot_nr);	
						//register reading
						AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
						int syncTime = vrf_rfIxReg.sharedReg.syncOffset;
						int timeDrift = vrf_rfIxReg.sharedReg.timeDrift;
						int freqDrift = vrf_rfIxReg.sharedReg.freqDrift; 
#ifdef MOD_STLC
						int slotOffset = vrf_rfIxReg.sharedReg.slotOffset; 
#endif
						AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

						//synch time command
						if (syncTime != 0)
						{
							vrf_syncInSlot(&timestamp, syncTime);
						}
						else
						{
							//time compensation command
							if (timeDrift >= VRF_TIMEDRIFT_DELTA)
							{
								deltaDrift = VRF_TIMEDRIFT_DELTA;
							}
							else if (timeDrift <= -VRF_TIMEDRIFT_DELTA)
							{
								deltaDrift = -VRF_TIMEDRIFT_DELTA;
							}
							
							if (deltaDrift != 0)
								LOG_D(PHY, "[VRF] ------- time compensation - %i (command:%i)\n", deltaDrift, timeDrift);
						}
#ifdef MOD_STLC
						if (slotOffset != 0)
						{
							LOG_E(VRF, "SLOT offset command %i -> slot nr : %i -> %i\n", slotOffset, slot_nr, (slot_nr+slotOffset)%VHW_NB_SAMPLEBANK);
							slot_nr = (slot_nr+slotOffset)%VHW_NB_SAMPLEBANK;
						}
#endif		
						//reset register commands
						AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
						vrf_rfIxReg.sharedReg.syncOffset = 0;
						vrf_rfIxReg.sharedReg.timeDrift = 0;
#ifdef MOD_STLC
						vrf_rfIxReg.sharedReg.slotOffset = 0;
#endif
						AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

			
						//frequency compensation command
						int inFreq = vrf_calc_freqOffsetIn(freqDrift);
					
						if (inFreq != 0)
						{
							vrf_rfDevcfg[0].digital_freq_offset += inFreq;
							vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

							LOG_D(PHY, "[VRF] ------- freq compensation - %i (command:%i, digital freq : %i)\n", inFreq, freqDrift, vrf_rfDevcfg[0].digital_freq_offset);
						}

						//reset register command
						AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
						vrf_rfIxReg.sharedReg.freqDrift = 0;
						AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");



						//slot decoding --------
						AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.roReg.regMutex)), "");
						vrf_rfIxReg.roReg.slot_nr = slot_nr;
						AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.roReg.regMutex)), "");
						
						if (vrf_checkRxStatus(slot_nr) == 0)
						{  	
							vrf_readSlot(&timestamp, slot_nr, deltaDrift);			
							vrf_irqRx(slot_nr);						
						} 
						else
						{
							LOG_E(PHY, "[WARNING] RX is in busy, so passing the slot %i\n", slot_nr);
							vrf_bypassSlot(&timestamp);						
						}

					}
					
					slot_nr++;
					slot_nr %= VHW_NB_SAMPLEBANK;
					
					break;
			
				case rfmode_tx:
					
					#ifdef VRF_DBGPROC
					vhw_timeStamp(&cur_time, "--> slot interval");
					vhw_initTime(&next_time);
					LOG_I(PHY, "[VRF] %i slot processing......\n", slot_nr);
					#endif
					
					//LOG_I(PHY, "[VRF] %i slot processing......\n", slot_nr);

					sched_slot_nr = (sched_slot_nr+1)%VHW_NB_SAMPLEBANK;

					
					//slot operation (TX or RX)
#ifdef MOD_STLC
					if (vrf_fdbkConfig.slotBmp & (0x01<<slot_nr)) //feedback slot
					{
						//Frequency offset compensation
						//register reading
						AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
						int freqDrift = vrf_rfIxReg.sharedReg.freqDrift; 
						AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

						
						int inFreq = vrf_calc_freqOffsetIn(freqDrift);
					
						if (inFreq != 0)
						{
							vrf_rfDevcfg[0].digital_freq_offset += inFreq;
							vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

							LOG_D(PHY, "[VRF] ------- freq compensation - %i (command:%i, digital freq : %i)\n", inFreq, freqDrift, vrf_rfDevcfg[0].digital_freq_offset);
						}

						//reset register command
						AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
						vrf_rfIxReg.sharedReg.freqDrift = 0;
						AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

						//register reading
						//RX the first part --------------------
						vrf_readFdbk(&timestamp, slot_nr);
						
						//scheduling gap -----------------------
						#ifdef VRF_DBGPROC						
						LOG_I(PHY, "[VRF] schedule IRQ for slot %i (fdbk), rest:%i\n", sched_slot_nr+vrf_slotDelay,vrf_fdbkConfig.offset);
						vhw_timeStamp(&next_time, " >>>> 3/4 rx : ");
						#endif
						
						vrf_irqTx(sched_slot_nr);
						usleep(vrf_SchedTime); //scheduling time

						//wakeup and quick sending next TX
						vrf_pushBuffer(&timestamp, vrf_samples_in_slot-vrf_fdbkConfig.offset, sched_slot_nr);
						#ifdef VRF_DBGPROC
						vhw_timeStamp(&next_time, " >>>> sendUSRP end : ");
						#endif
						}
					else //normal DL transmissionslot
#endif
					{
						vrf_bypassSample(&timestamp, vrf_samples_in_slot-vrf_IQSendMargin-vrf_SchedTimeSample);
						#ifdef VRF_DBGPROC						
						LOG_I(PHY, "[VRF] schedule IRQ for slot %i (normal), rest:%i, bitmap:%x\n", sched_slot_nr+vrf_slotDelay, vrf_IQSendMargin, vrf_rfIxReg.sharedReg.processBitmap);
						vhw_timeStamp(&next_time, " >>>> 3/4 rx : ");
						#endif

#ifdef MOD_STLC
						if (vrf_fdbkConfig.slotBmp & (0x01<<sched_slot_nr) == 0)
#endif
						{
							vrf_irqTx(sched_slot_nr);
							usleep(vrf_SchedTime);

							//wakeup and quick sending next TX
							vrf_pushBuffer(&timestamp, vrf_samples_in_slot, sched_slot_nr);
							#ifdef VRF_DBGPROC
							vhw_timeStamp(&next_time, " >>>> sendUSRP end : ");
							#endif
						}
					}
					vrf_bypassSample(&timestamp, vrf_IQSendMargin+vrf_SchedTimeSample);
					
					//slot count increment and interrupt
					slot_nr = (slot_nr+1)%VHW_NB_SAMPLEBANK;						
					//indicate the end of the TX/
					#ifdef VRF_DBGPROC
					vhw_timeStamp(&next_time, " >>>> receiveUSRP end : ");
					#endif

					break;
					
				default:
					break;			
			}	  	
		} //HW on loop

		if (vrf_rfHwOn == 1)
		{
			AssertFatal(vrf_rfdevice.trx_stop_func(&vrf_rfdevice) == 0, "Could not stop the RF device\n");
			vrf_changeHwStats(rfst_off);
			vrf_rfHwOn = 0;

			printf("[vHW] RF virtual HW is off!\n");
		}
	}// while !oai_exit

	if (vrf_rfdevice.trx_end_func)
		  vrf_rfdevice.trx_end_func(&vrf_rfdevice);

	return NULL;
}
