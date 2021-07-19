
#define _GNU_SOURCE
#include "vThread_RF.h"
#include "rt_wrapper.h"
#include "nr-uesoftmodem.h"
#include "../../ARCH/COMMON/common_lib.h"
#include "vThreadix_rf.h"

#include <sched.h>
#include <string.h>

#define VRF_FREQOFFSET_STEPSIZE		100
#define VRF_SEND_DELAY				4
#define VRF_TIMEDRIFT_DELTA			2

/* virtual H/W for RF 
- Assuming that there is only one RF in the UE
- virtual H/W has one real RF device and has HAL code for controling this real HW
*/


/*     ------- RF buffer ------          */
// TX/RX chain vHW interface
int32_t __attribute__ ((aligned (32))) vrf_rxdata[VHW_NB_SAMPLEBANK][HW_NB_RXANT][VHW_NB_MAXSAMPLES];
int32_t __attribute__ ((aligned (32))) vrf_txdata[VHW_NB_SAMPLEBANK][HW_NB_RXANT][VHW_NB_MAXSAMPLES];

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
	
	for (int i=0; i<HW_NB_RXANT; i++)
		rxp[i] = ((void *)&dummy_rx[i][0]);


	AssertFatal(vrf_samples_in_slot == 
				vrf_rfdevice.trx_read_func(&vrf_rfdevice,
				                           timestamp,
				                           rxp,
				                           vrf_samples_in_slot,
				                           HW_NB_RXANT),
				                           "");
}


//throw away samples till the next frame boundary
void vrf_syncInSlot(openair0_timestamp *timestamp, int sync_offset)
{
	void *rxp[HW_NB_RXANT];
	
	for (int i=0; i<HW_NB_RXANT; i++)
		rxp[i] = ((void *)&vrf_rxdata[0][i][0]);

	LOG_E(PHY, "[vRF] Resynchronizing RX by %d samples\n", sync_offset);

	while (sync_offset < 0)
	{
		sync_offset += vrf_samples_in_slot;
	}
	
	for ( int size = sync_offset ; size > 0 ; size -= vrf_samples_in_slot )
	{
  		int unitTransfer = size > vrf_samples_in_slot ? vrf_samples_in_slot : size ;
  		AssertFatal(unitTransfer == 
					vrf_rfdevice.trx_read_func(&vrf_rfdevice,
		                                         timestamp,
		                                         rxp,
		                                         unitTransfer,
		                                         HW_NB_RXANT),
												 "");
	}
}


void vrf_sendSlot(openair0_timestamp *timestamp, uint8_t slot_nr, uint8_t slot_offset)
{
	void *txp[HW_NB_TXANT];
	int writeBlockSize = vrf_samples_in_slot;

	for (int i=0; i<HW_NB_TXANT; i++)
		txp[i] = (void *)&vrf_txdata[slot_nr][i][0];

	AssertFatal( writeBlockSize ==
		   		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
							                   *timestamp +
							                   slot_offset*vrf_samples_in_slot,
							                   txp,
							                   writeBlockSize,
							                   HW_NB_TXANT,
							                   1),"");
}


//read one slot samples
void vrf_readSlot(openair0_timestamp *timestamp, uint8_t slot_nb, int deltaDrift)
{
	void *rxp[HW_NB_RXANT];

	for (int i=0; i<HW_NB_RXANT; i++)
		rxp[i] = ((void *)&vrf_rxdata[slot_nb][i][0]);

	AssertFatal( (vrf_samples_in_slot+deltaDrift )== 
				 vrf_rfdevice.trx_read_func(&vrf_rfdevice,
	                                        timestamp,
	                                        rxp,
	                                        vrf_samples_in_slot+deltaDrift,
	                                        HW_NB_RXANT), 
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


void vrf_irqTx(uint8_t slot_nr)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.roReg.regMutex)), "");
	vrf_rfIxReg.roReg.slot_nr = slot_nr;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.roReg.regMutex)), "");

	AssertFatal( 0 == pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
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
	vrf_rfIxReg.sharedReg.processBitmap= 0;
	//vrf_rfIxReg.sharedReg.txData = vrf_txdata;
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		
		for (int j=0;j<HW_NB_RXANT;j++)
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

	#if 0
	//immediate application of the offset command
	truncFreq = (freq_offset/VRF_FREQOFFSET_STEPSIZE)*VRF_FREQOFFSET_STEPSIZE;

	#else
	
	//drift command
	if (freq_offset > VRF_FREQOFFSET_STEPSIZE)
		truncFreq = VRF_FREQOFFSET_STEPSIZE;
	else if (freq_offset < -VRF_FREQOFFSET_STEPSIZE)
		truncFreq = -VRF_FREQOFFSET_STEPSIZE;
	#endif

	return truncFreq;
}



void *vRFIC_mainThread(void *arg)
{
	uint8_t onOffCfg = 0;
	openair0_timestamp timestamp;
	int i;
	char threadname[128];
	int rx_offset_diff;
	int slot_nr=-1;
	long long cur_time, next_time;

	vrf_rfIxReg.roReg.hwStatus = hwIx_null;

	//HW interface configuration
	// --------------- thread setting -------------------
	//CPU affinity setting (deleted if it is not needed)
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	if ( threads.main != -1 )
		CPU_SET(threads.main, &cpuset);
	sprintf(threadname, "RF virtual HW thread on UE side");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY, &cpuset, threadname);

	// -------------- HAL and HW interface configuration -------------------
	vrf_initHALInterface(arg);
	vrf_initRfInterface();

	//SDR RF initialization
	vrf_init_rfDevCfg();
	vrf_configInit();
	AssertFatal(0== openair0_device_load(&vrf_rfdevice, &vrf_rfDevcfg[0]), "");	
	vrf_rfdevice.host_type = RAU_HOST;




	
	vrf_changeHwStats(rfst_off);
	printf("[vHW] RF virtual HW is initialized, waiting for on signal\n");

	//outer loop for off waiting
	while (!oai_exit)
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
			time_stats_t onTime2;
			onTime2.p_time = rdtsc_oai();
			
		  	vrf_configInit();			
			AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.roReg.regMutex)), "");
			vrf_rfIxReg.roReg.slotSize = vrf_samples_in_slot;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.roReg.regMutex)), "");
			
			vrf_rfdevice.trx_configure_on(&vrf_rfdevice, vrf_rfDevcfg);
			AssertFatal(vrf_rfdevice.trx_start_func(&vrf_rfdevice) == 0, "Could not start the RF device\n");

			vrf_changeHwStats(rfst_on);
			vrf_rfHwOn = 1;

			onTime2.p_time = rdtsc_oai() - onTime2.p_time;
			printf("[vHW] >>>>>>>>>>>>>>>>> RF virtual HW is on! (carrier DL freq : %lf, mode:%i) - on time : %4d us\n", 
				vrf_rfDevcfg[0].rx_freq[0], vrf_rtCfg.mode, (int)(onTime2.p_time/cpuf/1000.0));
			cur_time = rdtsc_oai();

		}		
		
		//inner loop for on operation
		while ( !oai_exit && 
				 vrf_rfHwOn == 1 && 
				 (onOffCfg = vrf_readOnOff()) == 1 )
		{		
			//read the real-time configure register and apply in real-time
			uint8_t flag_rtcfg = vrf_configRealTime();
			int deltaDrift = 0;

			switch (vrf_rtCfg.mode)
			{
				case rfmode_rx:
								
					//register reading
					AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
					int syncTime = vrf_rfIxReg.sharedReg.syncOffset;
					int timeDrift = vrf_rfIxReg.sharedReg.timeDrift;
					int freqDrift = vrf_rfIxReg.sharedReg.freqDrift; 
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
							LOG_I(PHY, "[VRF] ------- time compensation - %i (command:%i)\n", deltaDrift, timeDrift);
					}
					
					//reset register commands
					AssertFatal ( 0== pthread_mutex_lock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");
					vrf_rfIxReg.sharedReg.syncOffset = 0;
					vrf_rfIxReg.sharedReg.timeDrift = 0;
					AssertFatal ( 0== pthread_mutex_unlock(&(vrf_rfIxReg.sharedReg.sharedMutex)), "");

		
					//frequency compensation command
					int inFreq = vrf_calc_freqOffsetIn(freqDrift);
				
					if (inFreq != 0)
					{
						vrf_rfDevcfg[0].digital_freq_offset += inFreq;
						vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

						LOG_I(PHY, "[VRF] ------- freq compensation - %i (command:%i, digital freq : %i)\n", inFreq, freqDrift, vrf_rfDevcfg[0].digital_freq_offset);
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
						vrf_bypassSlot(&timestamp);
			    	}
					slot_nr++;
					slot_nr %= VHW_NB_SAMPLEBANK;
					
					break;
			
				case rfmode_tx:

					vrf_bypassSlot(&timestamp);

					//start of the slot
					slot_nr++;
					slot_nr %= VHW_NB_SAMPLEBANK;
					
					//check processBitmap and send TX buffer
					vrf_sendSlot(&timestamp, slot_nr, VRF_SEND_DELAY);
					//indicate the end of the TX
					vrf_irqTx(slot_nr);
					
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
