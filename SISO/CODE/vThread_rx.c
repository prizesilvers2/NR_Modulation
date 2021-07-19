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


//i made
static int vrx_dmrsBuf[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32))); //demodulation symbols (before FFT / arrange)

static int vrx_dataBuf[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32))); //demodulation symbols (before FFT / arrange)

static int vrx_bfdataBuf[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32))); //demodulation symbols (after FFT / arrange)



//i made(for M-sequence)
static short vtx_fpreambleSeqBuf[2048] __attribute__ ((aligned (32)));



/* --------------- HAL configuration status type --------------- */

typedef struct { //initial configurations

	rxHwMode_e mode;				//one shot / continuous

	int preambleSeq[VRX_NB_MAX_PREAMBLE_SEQ] __attribute__ ((aligned (32)));

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
		int* ptr32;
		int* tx_ptr32;
		
		memset(vrx_preambleSeqBuf, 0, cfgInfo->fftSize*sizeof(int));
		memset(vtx_fpreambleSeqBuf, 0, cfgInfo->fftSize);
		
		ptr32 = vrx_preambleSeqBuf;
		tx_ptr32 = vtx_fpreambleSeqBuf;
		
		for(int i=0; i<cfgInfo->preamble_length/2; i++)
		{
			ptr32[i] = cfgInfo->preambleSeq[cfgInfo->preamble_length/2+i];
			ptr32[cfgInfo->fftSize - cfgInfo->preamble_length/2 +i] = cfgInfo->preambleSeq[i];
		}
					
		for(int i=0; i<cfgInfo->preamble_length; i++)
		{
			tx_ptr32[i+(cfgInfo->fftSize-cfgInfo->preamble_length)/2] = cfgInfo->preambleSeq[i];
		}

		
		vhw_idft( vrx_preambleSeqBuf ,&cfgInfo->preambleSeq , cfgInfo->fftSize);		
		cfgInfo->preamble_length = cfgInfo->fftSize ;

}






//generate the DMRS sequene and store it in the sequence buffer

void vrx_genDmrs(vrx_cfgInfo_t* cfgInfo)

{
	int prb_nr = cfgInfo->nbSubcarrier;
	int dmrsSymb[VHW_DMRS_NBSYMB] = VHW_DMRS_SYMB;

	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		for (int k=0;k<VHW_DMRS_NBSYMB;k++)
		{
			vhw_genReferenceSignalSeq((short*)&vrx_dmrsSeqBuf[i][k][0], i, dmrsSymb[k], prb_nr);
		}
	}
	
}

int Max(short x, short y, short z , short k)
{
	int max ;
    max = (x > y)? x : y;
    max = (max > z)? max : z;
    max = (max > k)? max : k;

	return max; 

}


static void vrx_decodeData(vrx_cfgInfo_t *cfgInfo, uint8_t slot_nr, uint8_t proc_nr, int time_offset)
{
	
	int dmrsSymb[VHW_DMRS_NBSYMB] = VHW_DMRS_SYMB;
	int k = 0;

	int vrx_dmrs_h[HW_NB_RE_IN_RB] ={0};

	int vrx_dmrs_start = 0;
	int vrx_data_start = 0;
	
	vrx_dmrs_start = time_offset- 3*(cfgInfo->fftSize+cfgInfo->nbCp);	
	vrx_data_start = time_offset- 6*(cfgInfo->fftSize+cfgInfo->nbCp);
	
	//dmrs//////////////////////////////////////////////////////////////////////////////////////////
	memset(vrx_dmrsBuf, 0, cfgInfo->fftSize);
	memset(vrx_demodBuf, 0, cfgInfo->fftSize);
	
	for(int i=0; i<cfgInfo->fftSize;i++)
	{	
		AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
		vrx_dmrsBuf[slot_nr][dmrsSymb[k]][i] = vrx_rfRegPtr->roReg.rxData[proc_nr][0][i+vrx_dmrs_start];
		AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	}

	vhw_dft(&vrx_dmrsBuf[slot_nr][dmrsSymb[k]][0], &vrx_demodBuf[slot_nr][dmrsSymb[k]][0], cfgInfo->fftSize);
				
	for(int i=0; i<HW_NB_RE_IN_RB; i++)
	{	
		vrx_dmrs_h[i]= vhw_calcConjMult((short*)(&vrx_dmrsSeqBuf[slot_nr][k][i]),(short*)(&vrx_demodBuf[slot_nr][dmrsSymb[k]][cfgInfo->fOffset_firstCarrier+i]),7);
	}
	
	//data///////////////////////////////////////////////////////////////////////////////////////////

	memset(vrx_bfdataBuf, 0, cfgInfo->fftSize);
	memset(vrx_dataBuf, 0, cfgInfo->fftSize);
	memset(vrx_decDataBuf, 0, cfgInfo->fftSize*2);
	
	
	for(int i=0; i<cfgInfo->fftSize;i++)
	{	
		AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
		vrx_bfdataBuf[slot_nr][dmrsSymb[k]][i] = vrx_rfRegPtr->roReg.rxData[proc_nr][0][i+vrx_data_start];
		AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	}

	vhw_dft(&vrx_bfdataBuf[slot_nr][dmrsSymb[k]][0], &vrx_dataBuf[slot_nr][dmrsSymb[k]][0], cfgInfo->fftSize);
		
	for(int i=0; i<cfgInfo->blockSize; i++)
	{
		vrx_decDataBuf[slot_nr][i]= vhw_calcConjMult((short*)(&vrx_dmrs_h[i]),(short*)(&vrx_dataBuf[slot_nr][dmrsSymb[k]][cfgInfo->fOffset_firstCarrier+i]),7);
		
	}
	
	printf("\n");
	printf("Decoding Data 10 \n");
	
	for(int i=0; i<cfgInfo->blockSize; i++)
	{
		printf("%d ", ((short*)(&vrx_decDataBuf[slot_nr][i]))[0]);
		printf("%d ", ((short*)(&vrx_decDataBuf[slot_nr][i]))[1]);
		
	}
	printf("\n");

	short symTable[8] = {1,1,1,-1,-1,1,-1,-1};
	short bitTable[4] = {11,10,01,00};

	int cor_result[5][4] ={0};

	short even_cor[5][4] = {0};
	
	short hd_index = 0;
	
	short decbit[5] ={0};

	int max = 0;

	short databit[10] ={0};

	short inputBit[10] = {1,0,1,1,0,0,0,1,1,1};
	
	//short inputBit[10] = {0,0,0,0,0,0,0,1,1,1};

	float BER = 0;

	int Nbit = 10;
	
	for (int i=0; i <cfgInfo->blockSize ; i++)
	{	
		for(int j=0; j<4; j++)
		{
			cor_result[i][j]= vhw_calcConjMult(&symTable[2*j],(short*)(&vrx_decDataBuf[slot_nr][i]),0);
		
			even_cor[i][j] = ((short*)(&cor_result[i][j]))[0];

			max = Max(even_cor[i][0],even_cor[i][1],even_cor[i][2],even_cor[i][3]);
			
			if(even_cor[i][j]==max)
			{
				hd_index = j;
			}							
		}
		
		decbit[i] = bitTable[hd_index];
		
	}

	printf("\n");
	printf("decode bit data \n");

	for(int i=0; i <cfgInfo->blockSize ; i++)
	{
		if(decbit[i] ==11)
		{
			databit[2*i] = 1;
			databit[2*i+1] = 1;
		}
		else if(decbit[i] ==10)
		{
			databit[2*i] = 1;
			databit[2*i+1] = 0;
		}
		else if(decbit[i] ==01)
		{
			databit[2*i] = 0;
			databit[2*i+1] = 1;
		}
		else
		{
			databit[2*i] = 0;
			databit[2*i+1] = 0;
		}
	}

	printf("%d %d %d %d %d %d %d %d %d %d", databit[0],databit[1],databit[2],databit[3],databit[4],databit[5],databit[6],databit[7],databit[8],databit[9]);
	
	printf("\n");


	int error[10] = {0};
	
	for(int i=0; i <cfgInfo->blockSize*2 ; i++)
	{
		error[i] = abs(databit[i]-inputBit[i]);
		
		if(error[i]>0.01)
		{
			BER += error[i];		
		}			
	}

	BER = BER/Nbit;
	
	printf("BER : %.2f \n", BER);
	
	printf("\n");
		
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









static int vrx_detectPreamble(vrx_cfgInfo_t* cfgInfo, uint8_t proc_nr, int *freqOffset, uint8_t state_sync)

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
			if (state_sync == 0)
			{
				if(n < (slotSize - cfgInfo->preamble_length))

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
			

			if (state_sync == 1)

			{
				// (cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize)
				if(((cfgInfo->nbCp0 + 7*cfgInfo->nbCp +5*cfgInfo->fftSize)<n)&&(n <(cfgInfo->nbCp0 + 7*cfgInfo->nbCp +7*cfgInfo->fftSize)))
				//if(2568<=n && n<=10760)
				//if(n < (slotSize - cfgInfo->preamble_length))
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
			

	}

	
	
		av_result /= cnt;

	// printf("state_sync == 0 , max_result : %d , max_offset : %d , Have to resynchronizing \n", max_result, max_offset);

	if (max_result < VRX_DEFAULT_THRES_PREAMBLEDETECTION*av_result)

		return(-1);



	if (abs( max_offset - ( cfgInfo->tOffset_preamble + cfgInfo->nbCp )) > cfgInfo->nbCp)

		LOG_D(PHY, "[VRX] WARNING ::: timing offset exceeds the cp length: %i, needs to be compensated immediately\n", max_offset - (cfgInfo->tOffset_preamble + cfgInfo->nbCp));
	
	

	f_off = vrx_estFreqOffsetFromPss((short*)&(cfgInfo->preambleSeq[0]), (short*) &(vrx_rfRegPtr->roReg.rxData[proc_nr][0][max_offset]), cfgInfo->preamble_length, shift);	

	LOG_E(PHY, "[VRX] Found preamble - offset : %i, max result : %li, bank : %i, frequency offset : %lf (%lf)\n", 

								max_offset, max_result, proc_nr, f_off, f_off*(15360000.0/cfgInfo->preamble_length));


	if(state_sync == 1)
	{
		printf("Doing window detection in state_sync =1 , This time value: %i \n", max_offset);
	}
	else
	{
		printf("Doing full detection in state_sync =0 , This time value: %i \n", max_offset);
	}
	
	*freqOffset = f_off*(15360000.0/cfgInfo->preamble_length);


	if (*freqOffset > 200 || *freqOffset < -200)

		LOG_D(PHY, "[VRX] WARNING ::: too large frequency offset : %lf, needs to be compensated immediately\n", *freqOffset);


	// vhw_dumpData(&(vrx_rfRegPtr->roReg.rxData[proc_nr][0][0]), 3*end, "firstdata.txt");
	

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
		int nbtime = 0;
		int overlap = 0;
		int slotSize = 15360;

		//inner loop for on operation ------------------------------------------------------

		while ( !oai_exit && 

				(vrx_HALix_out->hwStatus == rxst_on &&

				(onOffCfg = vrx_readOnOff()) == 1) )

		{

			vrx_checkRfIrq(procCnt); //wait until interrupt comes from RF

			vrx_configRealTime(&rtCfgInfo); //read the real-time configure register and apply in real-time



			//check preamble

			int freqOffset;

			
			if((status_synch==0) || slotCnt==2)
			{
				time_offset = vrx_detectPreamble(&cfgInfo, procCnt, &freqOffset, status_synch);
				
			}
			else
			{
				if(status_synch==1) //time_offset = 6664;
				{
					printf("-------------------------------------------------\n");
					printf("Decoding Data // proCnt = %d , slotCnt =%d // status_sych : %d \n", procCnt, slotCnt, status_synch);
					vrx_decodeData(&cfgInfo, slotCnt, procCnt, time_offset);
					vrx_setData(&vrx_decDataBuf[slotCnt][0], cfgInfo.blockSize , slotCnt);
				}
				else
				{
					printf("No detection time // proCnt = %d ,slotCnt =%d // status_sych : %d \n",procCnt, slotCnt, status_synch);
				}
				
				vrx_clear_processBitmap(procCnt);
				procCnt = (procCnt+1)%VHW_NB_SAMPLEBANK;
				slotCnt = (slotCnt+1)%VHW_NB_SAMPLEBANK;
				continue;
			}


			if(time_offset == -1)
			{			
				if((slotCnt ==2)&&(status_synch==1))							//여기까지 온 것이면, resynchronizing한 것임.
				{
					status_synch = 0;
					LOG_I(PHY, "window detection fail. State change 1->0 \n");
					
				}
				printf("time_offset == -1, proCnt : %d , slotCnt : %d \n",procCnt ,slotCnt);
				
				vrx_clear_processBitmap(procCnt);
				procCnt = (procCnt+1)%VHW_NB_SAMPLEBANK;
				slotCnt = (slotCnt+1)%VHW_NB_SAMPLEBANK;
				
				continue;
				
			}
			else if((slotCnt ==2)&&(status_synch==0)&&(overlap ==1))
			{
				nbtime = 1;												//After resynchronizing , go frequency offset change

			}
			
			if(status_synch==0)
			{
				if(nbtime ==1)
				{
					status_synch = 1;
				}
				else
				{
					AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
					vrx_rfRegPtr->sharedReg.syncOffset = time_offset - (cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize);
					AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
					LOG_I(PHY, "Configuring syncOffset by %i, statetus_synch ==0 \n" ,time_offset - (cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize));	

					printf("first proCnt : %d \n",procCnt);

					
					if(time_offset-(cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize)<0)
					{	
						procCnt = (procCnt+1)%VHW_NB_SAMPLEBANK;
						printf("syncOffset <0 , procCnt 1 count up , Now procCnt : %d \n", procCnt);
					}

					status_synch = 1;
					slotCnt = 2;
					overlap = 1;
					
					vrx_clear_processBitmap(procCnt);
					procCnt = (procCnt+1)%VHW_NB_SAMPLEBANK;
					slotCnt = (slotCnt+1)%VHW_NB_SAMPLEBANK;
					continue;
						
				}			
				
			}
			
		
			if(status_synch == 1)
			{	
				if(slotCnt==2)
				{	
					printf("Decoding Data // proCnt = %d , slotCnt =%d // status_sych : %d \n",procCnt, slotCnt, status_synch);
					vrx_decodeData(&cfgInfo, slotCnt, procCnt, time_offset);
					vrx_setData(&vrx_decDataBuf[slotCnt][0], cfgInfo.blockSize , slotCnt);
						
					AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
					vrx_rfRegPtr->sharedReg.freqDrift = freqOffset;
					AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");

					// when time_offset =! 6664
					if(time_offset != (cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize))
					{
						AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
						vrx_rfRegPtr->sharedReg.timeDrift= time_offset - (cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize) ;
						AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
						printf("time drift = %d \n", time_offset - (cfgInfo.nbCp0 + 7*cfgInfo.nbCp +6*cfgInfo.fftSize));						
					}
				}				
	
			}
			
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




