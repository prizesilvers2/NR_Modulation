#define _GNU_SOURCE
#include "vThread_rx.h"
#include "vThreadix_rf.h"
#include "nr-uesoftmodem.h"
#include "vThread_HWCommon.h"
#include "../../ARCH/COMMON/common_lib.h"

#define VRX_DEFAULT_THRES_PREAMBLEDETECTION		50
#define VRX_TIMEDRIFT_THRESH				4

#include <sched.h>
#include <string.h>

static short vrx_preambleSeqBuf[2*VRX_NB_MAX_PREAMBLE_SEQ] __attribute__ ((aligned (32))); //time-domain preamble sequence
static uint16_t vrx_preambleLen; //length of the time-domain preamble sequence
static int vrx_dmrsSeqBuf[VHW_NB_SAMPLEBANK][VHW_DMRS_NBSYMB][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32))); //DMRS sequence
static int vrx_demodBuf[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32))); //demodulation symbols (after FFT)
static int vrx_decDataBuf[VHW_NB_SAMPLEBANK][VHW_MAX_BLOCKSIZE]; //decoded data

/* --------------- HAL configuration status type --------------- */
typedef struct { //initial configurations
	rxHwMode_e mode;				//one shot / continuous
	int __attribute__ ((aligned (32))) preambleSeq[VRX_NB_MAX_PREAMBLE_SEQ];
	uint16_t preamble_length;
	uint32_t detect_window;
	uint32_t detect_offset;
	uint16_t fftSize;
	uint16_t nbCp;					//cyclic prefix length
	uint16_t nbCp0;					//cyclic prefix length (the first carrier)
	uint16_t fOffset_firstCarrier; 	//frequency subcarrier index offset for the first subcarrier
	uint32_t tOffset_preamble;		//time offset for the preamble
	uint16_t nbSubcarrier;			//number of subcarriers
	uint32_t blockSize;				//decoding data size

} vrx_cfgInfo_t;

typedef struct { //real-time configurations
	rxHwStatus_e status;
	uint8_t decIrq;
} vrx_rtCfg_t;


/*     ------- HAL - HW interface related ------          */
//registers
const halRx_woReg_t 		*vrx_HALix_in; 		//write only registers
halRx_roReg_t 				*vrx_HALix_out;		//read only registers
halRx_rwReg_t 				*vrx_HALix_buf;		//read only registers

pthread_mutex_t				*vrx_muPtr_outHal;	//MUTEX for HAL interface
pthread_mutex_t				*vrx_muPtr_inHal;	//MUTEX for HAL interface
pthread_mutex_t				*vrx_muPtr_bufHal;

pthread_cond_t				*vrx_csPtr_bufHal;	//condition signal for off -> on


/*     ------- HW - HW interface related ------          */
//RF - TX interface -- only one is defined, because there is assumed to be only one RF
vrfix_Reg_t* 				vrx_rfRegPtr;		//rf registers 

//generate preamble time samples
void vrx_genPreamble(vrx_cfgInfo_t* cfgInfo)
{	
	
}


//generate the DMRS sequene and store it in the sequence buffer
void vrx_genDmrs(vrx_cfgInfo_t* cfgInfo)
{
	
}


int8_t vrx_configInit(vrx_cfgInfo_t* cfgInfo, vrx_rtCfg_t* rtCfgInfo)
{
	cfgInfo->mode = vrx_HALix_in->mode;
	cfgInfo->preamble_length = vrx_HALix_in->preamble_length;
	memcpy(cfgInfo->preambleSeq, vrx_HALix_in->preambleSeq, cfgInfo->preamble_length*sizeof(int));	
	cfgInfo->detect_window = vrx_HALix_in->detect_window;
	cfgInfo->detect_offset = vrx_HALix_in->detect_offset;
	cfgInfo->fftSize = vrx_HALix_in->fftSize;
	cfgInfo->nbCp = ((int)VHW_DEFAULT_CPLENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->nbCp0 = ((int)VHW_DEFAULT_CP0LENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->fOffset_firstCarrier = vhw_calcFirstCarrier(cfgInfo->fftSize);
	cfgInfo->tOffset_preamble = (cfgInfo->fftSize+cfgInfo->nbCp)*VHW_PREAMBLE_SYMBOL + cfgInfo->nbCp0;
	cfgInfo->nbSubcarrier = vhw_calcNbSubcarrier(cfgInfo->fftSize);
	cfgInfo->blockSize = vrx_HALix_in->decBlockSize;

	//preamble calculation
	vrx_genPreamble(cfgInfo);
	vrx_genDmrs(cfgInfo);

	LOG_E(PHY, "[vRX] configured and will start : mode %i, preamble length : %i, FFT size : %i, CP:%i/%i, preamble offset:%i, window:%i, offset:%i, decoded length : %i\n",
		cfgInfo->mode, cfgInfo->preamble_length, cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0, cfgInfo->tOffset_preamble, cfgInfo->detect_window, cfgInfo->detect_offset, cfgInfo->blockSize);

	vrx_HALix_buf->decIrq = 0;
	rtCfgInfo->decIrq = 0;

	return 0;
}


static void vrx_configRealTime(vrx_rtCfg_t* rtCfgInfo)
{

}

double vrx_estFreqOffsetFromPss(short *preamble, short *rxdata, uint16_t seq_length, int shift)
{

	// fractional frequency offser computation according to Cross-correlation Synchronization Algorithm Using PSS
	// Shoujun Huang, Yongtao Su, Ying He and Shan Tang, "Joint time and frequency offset estimation in LTE downlink," 7th International Conference on Communications and Networking in China, 2012.
	int64_t result1,result2;
	// Computing cross-correlation at peak on half the symbol size for first half of data
	result1  = dot_product64(preamble, 
				  rxdata, 
				  (seq_length>>1), 
				  shift);
	// Computing cross-correlation at peak on half the symbol size for data shifted by half symbol size 
	// as it is real and complex it is necessary to shift by a value equal to symbol size to obtain such shift
	result2  = dot_product64(preamble+seq_length, 
				  rxdata+seq_length, 
				  (seq_length>>1), 
				  shift);

	int64_t re1,re2,im1,im2;
	re1=((int*) &result1)[0];
	re2=((int*) &result2)[0];
	im1=((int*) &result1)[1];
	im2=((int*) &result2)[1];

	// estimation of fractional frequency offset: angle[(result1)'*(result2)]/pi
	return (atan2(re1*im2-re2*im1,re1*re2+im1*im2)/VHW_PI);
}




static int vrx_detectPreamble(vrx_cfgInfo_t* cfgInfo, uint8_t proc_nr, int *freqOffset)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->roReg.regMutex)), "");
	int slotSize = vrx_rfRegPtr->roReg.slotSize;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->roReg.regMutex)), "");
	
	int end = slotSize;
	int64_t result = 0, max_result = 0;
	int64_t av_result = 0;
	int max_offset = 0;
	int cnt = 0;
	int16_t maxval = 0;
	int shift;
	int16_t *ptr16;
	double f_off;

	
	//long long cur_time = rdtsc_oai();
	if (cfgInfo->detect_window > 0 &&
		end > cfgInfo->detect_offset+cfgInfo->detect_window)
	{

		end = cfgInfo->detect_offset+cfgInfo->detect_window;
	}		

	ptr16 = (short*)&(cfgInfo->preambleSeq[0]);
	for (int i=0; i<2*cfgInfo->preamble_length; i++)
	{
		maxval = max(maxval,ptr16[i]);
		maxval = max(maxval,-ptr16[i]);
	}
	shift = vhw_log2Approx(maxval);
	
	for (int n=cfgInfo->detect_offset; n < end; n+=4)
	{ 
		if ( n < (slotSize - cfgInfo->preamble_length) )
		{
			int64_t dotResult	= dot_product64((short*)&(cfgInfo->preambleSeq[0]),
									(short*) &(vrx_rfRegPtr->roReg.rxData[proc_nr][0][n]), 
									  cfgInfo->preamble_length, 
									  shift);
			result = vhw_abs64(dotResult);
			av_result += result;
			
			/* calculate the absolute value of sync_corr[n] */
			if (result > max_result)
			{
				max_result = result;
				max_offset = n;
			}
			cnt++;
		}
		
	}


	av_result /= cnt;
	
	if (max_result < VRX_DEFAULT_THRES_PREAMBLEDETECTION*av_result)
		return(-1);

	if (abs( max_offset - ( cfgInfo->tOffset_preamble + cfgInfo->nbCp )) > cfgInfo->nbCp)
		LOG_D(PHY, "[VRX] WARNING ::: timing offset exceeds the cp length: %i, needs to be compensated immediately\n", max_offset - (cfgInfo->tOffset_preamble + cfgInfo->nbCp));

	f_off = vrx_estFreqOffsetFromPss((short*)&(cfgInfo->preambleSeq[0]), (short*) &(vrx_rfRegPtr->roReg.rxData[proc_nr][0][max_offset]), cfgInfo->preamble_length, shift);	
	LOG_E(PHY, "[VRX] Found preamble - offset : %i, max result : %li, bank : %i, frequency offset : %lf (%lf)\n", 
								max_offset, max_result, proc_nr, f_off, f_off*(15360000.0/cfgInfo->preamble_length));

	*freqOffset = f_off*(15360000.0/cfgInfo->preamble_length);

	if (*freqOffset > 200 || *freqOffset < -200)
		LOG_D(PHY, "[VRX] WARNING ::: too large frequency offset : %lf, needs to be compensated immediately\n", *freqOffset);

	return(max_offset);		
}




static uint8_t vrx_readOnOff(void)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_bufHal), "error on RX HAL MUTEX while reading onOff");
	onOff = vrx_HALix_buf->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_bufHal), "error on RX HAL MUTEX while reading onOff");

	return onOff;
}


static void vrx_checkOnOff(void)
{
	AssertFatal ( 0 == pthread_mutex_lock(vrx_muPtr_bufHal), "");
	while (vrx_HALix_buf->onoff == 0)
		  pthread_cond_wait( vrx_csPtr_bufHal, vrx_muPtr_bufHal );
	AssertFatal ( 0 == pthread_mutex_unlock(vrx_muPtr_bufHal), "");
}



static void vrx_forceOff(void)
{
	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_bufHal), "error on RX HAL MUTEX while forcing onOff");
	vrx_HALix_buf->onoff = 0;
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_bufHal), "error on RX HAL MUTEX while forcing onOff");
}

static void vrx_indicate_rx(uint8_t slot_nr, int time_offset)
{
	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_outHal), "");
	vrx_HALix_out->slot_nr = slot_nr;
	vrx_HALix_out->tOffset = time_offset;
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");

	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_bufHal), "");
	vrx_HALix_buf->decIrq = 1;
	AssertFatal( 0 == pthread_cond_signal(vrx_csPtr_bufHal), "");
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_bufHal), "");
}


static void vrx_checkRfIrq(uint8_t procCnt)
{
	//interrupt from RF
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	while ( (vrx_rfRegPtr->sharedReg.processBitmap & (1<<procCnt))  == 0)
		pthread_cond_wait( &(vrx_rfRegPtr->sharedReg.irq_txrxInst), &(vrx_rfRegPtr->sharedReg.sharedMutex) ); // the thread waits here most of the time
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
}


static void vrx_clear_processBitmap(uint8_t procCnt)
{
	vrx_rfRegPtr->sharedReg.processBitmap &= (~(1<<procCnt));
}


static void vrx_set_processBitmapOn(void)
{
	vrx_rfRegPtr->sharedReg.processBitmap |= VRF_PROCBITMAP_MASK_ONOFF;
}

static void vrx_clear_processBitmapOn(void)
{
	vrx_rfRegPtr->sharedReg.processBitmap &= ~VRF_PROCBITMAP_MASK_ONOFF;
}

static void vrx_setData(int* dataPtr, int size, int slot_nr)
{
	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_outHal), "");
	memcpy(&(vrx_HALix_out->data[slot_nr][0]), dataPtr, size*sizeof(int));
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");
}



void *vRX_mainThread(void *arg)
{
	static int __thread UE_thread_rx_retval;
	char threadname[128];
	
	uint8_t onOffCfg;
	vrx_cfgInfo_t cfgInfo;
	vrx_rtCfg_t rtCfgInfo;
	uint8_t procCnt = 0, slotCnt = 0;
	hwIxStatus_e rfst;
	int time_offset;
	uint8_t status_synch = 0;

	
	//register initialization ------------------------------------
	vrx_HALix_in 	= &( ((ix_halRx_t*)arg)->woReg );
	vrx_HALix_out 	= &( ((ix_halRx_t*)arg)->roReg );
	vrx_HALix_buf 	= &( ((ix_halRx_t*)arg)->rwReg );

	vrx_muPtr_outHal = (pthread_mutex_t*)&(((ix_halRx_t*)arg)->roReg.mutex_roHal);
	vrx_muPtr_inHal = (pthread_mutex_t*)&(((ix_halRx_t*)arg)->woReg.mutex_woHal);
	vrx_muPtr_bufHal = (pthread_mutex_t*)&(((ix_halRx_t*)arg)->rwReg.mutex_rwHal); 
	
	vrx_csPtr_bufHal = (pthread_cond_t*)&(((ix_halRx_t*)arg)->rwReg.cond_rwHal);
	
	vrx_HALix_out->hwStatus = rxst_null;

	//HW interface initialization
	// vs. RF
	AssertFatal ( NULL != (vrx_rfRegPtr = vrf_configRfReg()), "[vHW][ERROR] error in configuration RF-RX register : pointer is NULL!\n");

	//thread initialization
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	if ( threads.sync != -1 )
		CPU_SET(threads.sync, &cpuset);
	// this thread priority must be lower that the main acquisition thread
	sprintf(threadname, "Virtual RX");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY-1, &cpuset, threadname);

	printf("[vHW] RX virtual HW is initialized, waiting for on signal \n");

	do
	{
		usleep(100);
		AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->roReg.regMutex)), "");
		rfst = vrx_rfRegPtr->roReg.hwStatus;
		AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->roReg.regMutex)), "");
		
	} while(rfst == hwIx_null);

	printf("[vHW] RX virtual HW is starting now \n");

	vrx_HALix_out->hwStatus = rxst_off;
	vrx_HALix_out->length = 0;
	
	while (!oai_exit)
	{
		//off state loop (stay until on signal comes) ---------------------------------------
		vrx_checkOnOff();
		
		//check register and process it
		//if register is 'on', then do static configuration (init and start)
		vrx_configInit(&cfgInfo, &rtCfgInfo);
		//turn the register value into 'on'
		AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_outHal), "");
		vrx_HALix_out->hwStatus = rxst_on;
		AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");
		//indicate to RF
		vrx_set_processBitmapOn();
		status_synch = 0;
		
		//inner loop for on operation ------------------------------------------------------
		while ( !oai_exit && 
				(vrx_HALix_out->hwStatus == rxst_on &&
				(onOffCfg = vrx_readOnOff()) == 1) )
		{
			vrx_checkRfIrq(procCnt); //wait until interrupt comes from RF
			vrx_configRealTime(&rtCfgInfo); //read the real-time configure register and apply in real-time

			//check preamble
			int freqOffset;
			time_offset = vrx_detectPreamble(&cfgInfo, procCnt, &freqOffset)

			vrx_clear_processBitmap(procCnt);
			procCnt = (procCnt+1)%VHW_NB_SAMPLEBANK;
			slotCnt = (slotCnt+1)%VHW_NB_SAMPLEBANK;
		}

		AssertFatal ( 0 == pthread_mutex_lock(vrx_muPtr_outHal), "");
		vrx_HALix_out->hwStatus = rxst_off;
		AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");

	}  // while !oai_exit

	return &UE_thread_rx_retval;
}

