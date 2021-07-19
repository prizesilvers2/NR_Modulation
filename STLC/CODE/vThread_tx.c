#define _GNU_SOURCE
#include "vThread_tx.h"
#include "NPAL_thread.h"
#include "NPAL_engines.h"
#include "vThread_HWCommon.h"
#include "vTRX_dmrs.h"
#include "vThreadix_rf.h"

#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef BUILDOPT_MACOS
#include <malloc.h>
#endif

//#define VTX_DBGPROC

#ifdef	MOD_STLC
#define VTX_SYNC_WINDOW_SIZE					1000
#define VTX_ONLINE_THRES_PREAMBLEDETECTION		2
#define VTX_DEFAULT_THRES_PREAMBLEDETECTION		20
#endif

#define bitsend 
#define dumpSize				100
#define VTX_DUMPFORCSI

//#define VTX_DBGSLOTDUMP
//#define VTX_SLOTDUMPLEN		800


/* --------------- HAL configuration status type --------------- */
typedef struct { //initial configurations
	hw_trxMode_e mode;				
	int preambleSeq[VTX_NB_MAX_PREAMBLE_SEQ];
	uint32_t preamble_length;
	uint16_t fftSize;
	uint16_t nbCp;
	uint16_t nbCp0;
	uint16_t fOffset_firstCarrier;
	uint32_t tOffset_preamble;
	uint16_t nbSubcarrier;
#if (defined MOD_STBC || defined MOD_STLC)
	int TXAnt_nb;	
#endif
} vtx_cfgInfo_t;

typedef struct { //real-time configurations
	txHwStatus_e status;
	uint32_t length[VHW_NB_SAMPLEBANK];				//center frequency
	void* txData[VHW_NB_SAMPLEBANK];					//u value
	uint8_t slot_nr_sch;
	uint32_t slotSize;
} vtx_rtCfg_t;

/*    ------- BUFFER ----------------------               */
#ifdef MOD_STLC
static int vrx_demodBuf[HW_NB_TXANT][VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vrx_preambleDump[HW_NB_TXANT][500*VHW_NB_SAMPLEBANK][VHW_DEFAULT_FFT_SIZE] __attribute__ ((aligned (32)));
static int vrx_Dump[HW_NB_TXANT][500*VHW_NB_SAMPLEBANK][6590] __attribute__ ((aligned (32)));
#endif

#if (defined MOD_STBC || defined MOD_STLC)
static int vtx_buffer[HW_NB_TXANT][VHW_NB_SAMPLEBANK][VHW_NB_MAXSAMPLES] __attribute__ ((aligned (32)));
static int vtx_freqBuffer[HW_NB_TXANT][VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vtx_freqBufferBp[HW_NB_TXANT][VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT] __attribute__ ((aligned (32)));
static int vtx_precodeBuffer[2*HW_NB_TXANT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));

#else
static int vtx_buffer[VHW_NB_SAMPLEBANK][VHW_NB_MAXSAMPLES] __attribute__ ((aligned (32)));
static int vtx_freqBuffer[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vtx_freqBufferBp[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT] __attribute__ ((aligned (32)));
#endif

static int vtx_dmrsBp[HW_NB_SYM_IN_SLOT][VHW_NB_MAXPRB] __attribute__ ((aligned (32)));


/*     ------- HAL - HW interface related ------          */
//registers
const halTx_woReg_t 		*vtx_HALix_in; 		//write only registers
halTx_roReg_t 				*vtx_HALix_out;		//read only registers
halTx_rwReg_t 				*vtx_HALix_buf;		//read only registers    //read & write ..? 

pthread_mutex_t				*vtx_muPtr_outHal;	//MUTEX for HAL interface
pthread_mutex_t				*vtx_muPtr_inHal;	//MUTEX for HAL interface
pthread_mutex_t				*vtx_muPtr_bufHal;

pthread_cond_t				*vtx_csPtr_inHal;	//condition signal for off -> on, data buffer read command
pthread_cond_t				*vtx_csPtr_slotIrqHal;
pthread_cond_t				*vtx_csPtr_bufHal;



/*     ------- HW - HW interface related ------          */
//RF - TX interface -- only one is defined, because there is assumed to be only one RF
vrfix_Reg_t* 				vtx_rfRegPtr;		//rf registers seen by srcher banks


//reset all the buffers
//not considering the preamble, DMRS, etc.. just set all to 0
void vtx_resetBufs(void)
{
#if (defined MOD_STBC || defined MOD_STLC)
	for (int nAnt=0;nAnt<HW_NB_TXANT;nAnt++)
#endif	
	{
		for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
		{
			for (int j=0;j<HW_NB_SYM_IN_SLOT;j++)
			{
#if (defined MOD_STBC || defined MOD_STLC)
				memset(&vtx_freqBuffer[nAnt][i][j][0], 0, VHW_NB_MAXFFTSIZE*sizeof(int)); //TX Ant 0 freq buffer, reset the frequence buffer
				vtx_freqBufferBp[nAnt][i][j] = 0;
#else
				memset(&vtx_freqBuffer[i][j][0], 0, VHW_NB_MAXFFTSIZE*sizeof(int)); //TX Ant 0 freq buffer, reset the frequence buffer				
				vtx_freqBufferBp[i][j] = 0;
#endif
				
			}

#if (defined MOD_STBC || defined MOD_STLC)
			memset(&vtx_buffer[nAnt][i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int)); //TX Ant 0 buffer, reset the time buffer
#else
			memset(&vtx_buffer[i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int)); //TX Ant 0 buffer, reset the time buffer
#endif
		}
	}

	for (int j=0;j<HW_NB_SYM_IN_SLOT;j++)
	{
		for (int k=0;k<VHW_NB_MAXPRB;k++)
		{
			vtx_dmrsBp[j][k] = 0;
		}
	}

}

//clear the frequency buffers
//slot bitmap : clear the buffer (except preamble and DMRS parts)
void vtx_clearFreqBuf(vtx_cfgInfo_t *cfgInfo, uint8_t slotBitmap)
{
#if (defined MOD_STBC || defined MOD_STLC)
	for (int AntID=0;AntID<cfgInfo->TXAnt_nb;AntID++)
#endif
	{
		for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
		{
			if (slotBitmap & (0x01 << i)) //if it is given to clear the buffer for the slot
			{
				for (int k=0;k<HW_NB_SYM_IN_SLOT;k++)
				{
					if (i == VHW_PREAMBLE_SLOT && k == VHW_PREAMBLE_SYMBOL) //preamble OFDM symbol : skip it
					{
						continue;
					}
#if (defined MOD_STBC || defined MOD_STLC)
					memset(&vtx_freqBuffer[AntID][0][i][k], 0, VHW_NB_MAXFFTSIZE*sizeof(int)); // TX Ant freq buffer , reset the buffer set for slot i and symbol k
					vtx_freqBufferBp[AntID][i][k] = 0;
#else
					memset(&vtx_freqBuffer[0][i][k], 0, VHW_NB_MAXFFTSIZE*sizeof(int)); // TX Ant freq buffer , reset the buffer set for slot i and symbol k
					vtx_freqBufferBp[i][k] = 0;
#endif
				}
			}
		}	
		
	}

	for (int j=0;j<HW_NB_SYM_IN_SLOT;j++)
	{
		for (int k=0;k<cfgInfo->nbSubcarrier/HW_NB_RE_IN_RB;k++)
		{
			vtx_dmrsBp[j][k] = 0;
		}
	}
}

//clear the time buffers
//slot bitmap : clear the buffer (except preamble parts)

void vtx_clearBuf(vtx_cfgInfo_t* cfgInfo, uint8_t slotBitmap)
{
#if (defined MOD_STBC || defined MOD_STLC)
	for (int AntID=0;AntID<cfgInfo->TXAnt_nb;AntID++)
#endif
	{
		for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
		{
			if (slotBitmap & (0x01 << i))
			{
				if ( i != VHW_PREAMBLE_SLOT )
				{
#if (defined MOD_STBC || defined MOD_STLC)
					memset(&vtx_buffer[AntID][i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int));		//TX Ant time buffer
#else
					memset(&vtx_buffer[i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int));		//TX Ant time buffer
#endif
				}
					
				else //clear only the time sample part that is not for preamble OFDM symbol
				{
#if (defined MOD_STBC || defined MOD_STLC)
					memset(&vtx_buffer[AntID][i][0], 0, (cfgInfo->tOffset_preamble-1)*sizeof(int));
					memset(&vtx_buffer[AntID][i][cfgInfo->tOffset_preamble+cfgInfo->nbCp+cfgInfo->fftSize], 0, (VHW_NB_MAXSAMPLES-cfgInfo->tOffset_preamble-cfgInfo->nbCp-cfgInfo->fftSize)*sizeof(int));
#else
					memset(&vtx_buffer[i][0], 0, (cfgInfo->tOffset_preamble-1)*sizeof(int));
					memset(&vtx_buffer[i][cfgInfo->tOffset_preamble+cfgInfo->nbCp+cfgInfo->fftSize], 0, (VHW_NB_MAXSAMPLES-cfgInfo->tOffset_preamble-cfgInfo->nbCp-cfgInfo->fftSize)*sizeof(int));
#endif
				}
			}
		}
	}
}


//modulate symbol for TX
//fftSize, nbCp, nbCp0 : frame parameters (fft size, CP length, CP0 length)
//symbol : OFDM symbol index (0~13)
//Ns : slot index
int vtx_modSymbol(			uint16_t fftSize, 		//fft size
							uint16_t nbCp, 			//number of cyclic prefix
							uint16_t nbCp0, 		//number of cyclic prefix for the first OFDM symbol
							unsigned char symbol, 	//symbol number for processing IFFT
							unsigned char Ns	  	//slot number for processing IFFT	
#if (defined MOD_STBC || defined MOD_STLC)
							,
							int AntID				//Ant ID for MIMO
#endif
				 		)
{
	unsigned int tOffset, tIfftOffset;
	int vtx_timeBuf[HW_MAX_FFTSIZE] __attribute__ ((aligned (32)));

	if (Ns<0 || Ns>=VHW_NB_SAMPLEBANK)
	{
		LOG_E(VTX, "IFFT : Ns must be between 0 and %i\n", VHW_NB_SAMPLEBANK);
		return(-1);
	}
	if (symbol<0 || symbol>=HW_NB_SYM_IN_SLOT)
	{
		LOG_E(VTX, "IFFT : Ns must be between 0 and %i\n", VHW_NB_SAMPLEBANK);
		return(-1);
	}
#if (defined MOD_STBC || defined MOD_STLC)	
	if (AntID >= HW_NB_TXANT)
	{
		LOG_E(VTX, "IFFT : ANT ID must be between 0 and %i\n", HW_NB_TXANT);
		return(-1);
	}
#endif

	//insertion offset calcluation
	if (symbol == 0)
	{
		tOffset = 0;
		tIfftOffset = nbCp0+nbCp;
	}
	else
	{
		tOffset = nbCp0;
		tOffset += symbol*(fftSize+nbCp);
		tIfftOffset = tOffset + nbCp;
	}

	//ifft freq samples --> time samples
	//clear the output time-domain buffer first :: for preventing overflow
#if (defined MOD_STBC || defined MOD_STLC)

	vhw_idft(&vtx_freqBuffer[AntID][Ns][symbol][0], &vtx_timeBuf[0], fftSize);

	memcpy(&vtx_buffer[AntID][Ns][tIfftOffset], &vtx_timeBuf[0], (fftSize)*sizeof(int));
	//CP insertion
	if (symbol == 0)
	{
		memcpy(&vtx_buffer[AntID][Ns][tOffset], &vtx_buffer[AntID][Ns][tIfftOffset+fftSize - (nbCp0+nbCp)], (nbCp0+nbCp)*sizeof(int));
	}
	else
	{
		memcpy(&vtx_buffer[AntID][Ns][tOffset], &vtx_buffer[AntID][Ns][tIfftOffset+fftSize - nbCp], nbCp*sizeof(int));
	}	
#else
	vhw_idft(&vtx_freqBuffer[Ns][symbol][0], &vtx_timeBuf[0], fftSize);
	memcpy(&vtx_buffer[Ns][tIfftOffset], &vtx_timeBuf[0], (fftSize)*sizeof(int));
	//CP insertion
	if (symbol == 0)
	{
		memcpy(&vtx_buffer[Ns][tOffset], &vtx_buffer[Ns][tIfftOffset+fftSize - (nbCp0+nbCp)], (nbCp0+nbCp)*sizeof(int));
	}
	else
	{
		memcpy(&vtx_buffer[Ns][tOffset], &vtx_buffer[Ns][tIfftOffset+fftSize - nbCp], nbCp*sizeof(int));
	}	
#endif
	
	return(0);
}


//modulation process : generate time buffer from freq buffer
//assumption : freq buffer has already generated
//Ns : slot number
//symbol bitmap : symbol to generate
int vtx_modSlot(vtx_cfgInfo_t* cfgInfo, 
#if (defined MOD_STBC || defined MOD_STLC)
					 int AntID, 
#endif
					 uint8_t Ns, uint16_t symbolBitmap)
{
	//modulation process for each OFDM symbol in the slot Ns
	for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
	{
		if ((symbolBitmap & (0x01 << i)) > 0) // if it is given to modulation it
		{
#if (defined MOD_STBC || defined MOD_STLC)
			vtx_modSymbol(cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0,  i, Ns, AntID);
#else
			vtx_modSymbol(cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0,  i, Ns);
#endif
		}
	}

	return 0;
}


//insert the generated DMRS sequence to the frequency buffer
//slot nr : slot number
//prb nr : physical RB number (1RB : 12 subcarriers)
//dmrs nr : DMRS symbol number (in time)
void vtx_insertDmrs(vtx_cfgInfo_t* cfgInfo, 
#if (defined MOD_STBC || defined MOD_STLC)
						  int AntID, 
#endif
						  int slot_nr, int prb_nr, int symb_nr) // allocation unit : 12 sub (1 RB)
{
	int fOffset = (cfgInfo->fOffset_firstCarrier + 12*prb_nr)%cfgInfo->fftSize;			//1 prb(12 subcarrier) unit
	int dmrs_nr;
	int dmrs_symb_nr;

#if (defined MOD_STBC || defined MOD_STLC)
	dmrs_symb_nr = vtrx_getNextDmrsSymbNb(symb_nr, AntID, &dmrs_nr);
#else
	dmrs_symb_nr = vtrx_getNextDmrsSymbNb(symb_nr, &dmrs_nr);
#endif	
	
	if (dmrs_symb_nr >= 0 && vtx_dmrsBp[dmrs_symb_nr][prb_nr] == 0)
	{
		for (int i=0;i<HW_NB_RE_IN_RB;i++)
		{
			int subc_idx = HW_NB_RE_IN_RB*prb_nr+i;
#if (defined MOD_STBC || defined MOD_STLC)
			vtx_freqBuffer[AntID][slot_nr][dmrs_symb_nr][(fOffset+i)%cfgInfo->fftSize] = vtrx_getDmrsSymb(slot_nr, dmrs_nr, subc_idx);//vtx_dmrsSeqBuf[slot_nr][dmrs_nr][12*prb_nr+i];
			vtx_freqBufferBp[AntID][slot_nr][dmrs_symb_nr] = 1;	//mark as the frequency buffer is filled out
#else
			vtx_freqBuffer[slot_nr][dmrs_symb_nr][(fOffset+i)%cfgInfo->fftSize] = vtrx_getDmrsSymb(slot_nr, dmrs_nr, subc_idx);//vtx_dmrsSeqBuf[slot_nr][dmrs_nr][12*prb_nr+i];
			vtx_freqBufferBp[slot_nr][dmrs_symb_nr] = 1;	//mark as the frequency buffer is filled out
#endif
		}
		LOG_D(VTX, "allocated DMRS at symb:%i, prb:%i (%i - %i), start subc : %i\n", dmrs_symb_nr, prb_nr, HW_NB_RE_IN_RB*prb_nr, HW_NB_RE_IN_RB*prb_nr+11, fOffset);
		
		vtx_dmrsBp[dmrs_symb_nr][prb_nr] = 1;
	}
}


void vtx_insertPreamble(vtx_cfgInfo_t* cfgInfo)
{
	uint16_t symbol_bitmap = (0x01 << VHW_PREAMBLE_SYMBOL);
	int k = cfgInfo->fftSize - cfgInfo->preamble_length/2;

	for (int i=0;i<cfgInfo->preamble_length;i++)
	{
#if (defined MOD_STBC || defined MOD_STLC)
		vtx_freqBuffer[0][VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL][k] = cfgInfo->preambleSeq[i];
		vtx_freqBufferBp[0][VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL] = 1;

		if (cfgInfo->TXAnt_nb > 1)
		{
			vtx_freqBuffer[1][VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL][k] = 0;
			vtx_freqBufferBp[1][VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL] = 1;
		}
#else
		vtx_freqBuffer[VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL][k] = cfgInfo->preambleSeq[i];  
		vtx_freqBufferBp[VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL] = 1;  		
#endif
		k = (k+1)%cfgInfo->fftSize;   
	}

	//clear the time buffer first
	vtx_clearBuf(cfgInfo, (0x01 << VHW_PREAMBLE_SLOT));

#if (defined MOD_STBC || defined MOD_STLC)
	for (int i=0;i<cfgInfo->TXAnt_nb;i++)
#endif
		vtx_modSlot(cfgInfo, 
#if (defined MOD_STBC || defined MOD_STLC)
					i, 
#endif
					VHW_PREAMBLE_SLOT, symbol_bitmap);


}


#ifdef MOD_STLC

static void vtx_commandFreqDrift(int freqDrift)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
	vtx_rfRegPtr->sharedReg.freqDrift = freqDrift;
	AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
}



double vtx_estFreqOffsetFromPss(short *preamble, short *rxdata, uint16_t seq_length, int shift)
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

	//estimation of fractional frequency offset: angle[(result1)'*(result2)]/pi
	return (atan2(re1*im2-re2*im1,re1*re2+im1*im2)/VHW_PI);
}



//#define CORR_DOWNSCALE		10
//need to be verified
static int vtx_detectPreamble (vtx_cfgInfo_t* cfgInfo, 
							   uint8_t proc_nr, 
							   int *freqOffset, 	
							   int detect_offset,
							   int state_sync) 

{
	AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->roReg.regMutex)), "");
	int slotSize = vtx_rfRegPtr->roReg.slotSize;
	AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->roReg.regMutex)), "");
	
	//modify window Size
	int start = 6800;		//6400
	int end = 7248;			//7000
	
	int64_t result = 0, max_result = 0;
	int64_t av_result = 0;
	int max_offset = 0; 
	int cnt = 0;
	int16_t maxval = 0;
	int shift;
	int16_t *ptr16;
	double f_off;
#ifdef VRX_CORHISTDUMP
	int64_t corr_hist[10000];
#endif

	LOG_D(PHY, "preamble detection trial in state %i and slot %i\n", state_sync, proc_nr);

	short vrx_preambleSeqBuf[2048];
	int vrx_preambleLen = 1024;

	memcpy(vrx_preambleSeqBuf,(short *)(&vtx_buffer[0][2][cfgInfo->tOffset_preamble]), cfgInfo->fftSize*sizeof(short)*2);
	
	//amplitude estimation -------------------------------------------------	
	ptr16 = &(vrx_preambleSeqBuf[0]);
	for (int i=0; i<2*vrx_preambleLen; i++)
	{
		maxval = max(maxval,ptr16[i]);
		maxval = max(maxval,-ptr16[i]);
	}
	shift = vhw_log2Approx(maxval);

	int VTX_WINDOW_Size = 300;
	
	//window search command
	if (detect_offset > 0)
	{
		start = detect_offset - VTX_WINDOW_Size;
		end = detect_offset + VTX_WINDOW_Size;
	}
	
	//-----------------------------------------------------------------------

	#ifdef VTX_DBGPROC	
	LOG_I(PHY, "preamble detection start :::::: start : %i, end : %i\n", start, end);
	#endif
	for (int n=start; n < end; n+=4) // n= 0 ~ end-4 (slotSize)
	{ 
		if ( n < (slotSize - vrx_preambleLen) )
		{
			int64_t dotResult	= dot_product64(&(vrx_preambleSeqBuf[0]),
								(short*) &(vtx_rfRegPtr->roReg.rxData[proc_nr][0][n]), 
								  vrx_preambleLen, 
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

	LOG_D(VTX, "[detection result] max_offset : %i \n" , max_offset);		
	

	if ((detect_offset == 0 && max_result < VTX_DEFAULT_THRES_PREAMBLEDETECTION*av_result) ||
		(detect_offset > 0 && max_result < VTX_ONLINE_THRES_PREAMBLEDETECTION*av_result))
	{
		LOG_D(VRX, "peak-to-average is below threshold! (max:%li av:%li) (start:%i, end:%i) end with assertion!\n", max_result, av_result, start, end);
		return(-1);
	}

	//f_off estmation :: angle, digital freq -----------------------------------
	f_off = vtx_estFreqOffsetFromPss(&(vrx_preambleSeqBuf[0]), (short*) &(vtx_rfRegPtr->roReg.rxData[proc_nr][0][max_offset]), vrx_preambleLen, shift);	
	
	//freqOffset = analog freq
	*freqOffset = (int)(f_off*(15360000.0/vrx_preambleLen));

	if (start == 0) // if it is full search
		LOG_I(PHY, "[VRX] Found preamble - offset : %i, max result : %li, av_result : %li, frequency offset : %lf (%lf)\n", 
								max_offset, max_result, av_result, f_off, f_off*(15360000.0/vrx_preambleLen));

	if (*freqOffset > 300 || *freqOffset < -300)
		LOG_E(PHY, "[VRX] WARNING ::: too large frequency offset : %i, needs to be compensated immediately\n", *freqOffset);

	
	return(max_offset);	

	
}
#endif




int8_t vtx_configInit(vtx_cfgInfo_t* cfgInfo, vtx_rtCfg_t* rtCfgInfo)
{

	cfgInfo->mode = vtx_HALix_in->mode;

	//frame structure definition
	cfgInfo->fftSize = vtx_HALix_in->fftSize;
	cfgInfo->nbCp = ((int)VHW_DEFAULT_CPLENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->nbCp0 = ((int)VHW_DEFAULT_CP0LENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->fOffset_firstCarrier = vhw_calcFirstCarrier(cfgInfo->fftSize);
	cfgInfo->tOffset_preamble = (cfgInfo->fftSize+cfgInfo->nbCp)*VHW_PREAMBLE_SYMBOL + cfgInfo->nbCp0;
	cfgInfo->nbSubcarrier = vhw_calcNbSubcarrier(cfgInfo->fftSize);

#if (defined MOD_STBC || defined MOD_STLC)
	//Antenna definition 
	if (cfgInfo->mode == hw_trxMode_siso || cfgInfo->mode == hw_trxMode_1x2STLC)
		cfgInfo->TXAnt_nb = 1;
	else if (cfgInfo->mode == hw_trxMode_2x1STBC)
		cfgInfo->TXAnt_nb = 2;
#endif

	//preamble definition
	cfgInfo->preamble_length = vtx_HALix_in->preamble_length;
	memcpy(&(cfgInfo->preambleSeq[0]), vtx_HALix_in->preambleSeq, vtx_HALix_in->preamble_length * sizeof(int));

	vtx_resetBufs();
	
	//preamble insertion
	vtx_insertPreamble(cfgInfo);

	//DMRS generation for 2 TX antenna
	vtrx_genDmrs(cfgInfo->nbSubcarrier,
#if (defined MOD_STBC || defined MOD_STLC)
				cfgInfo->mode,
#endif
				VTRX_DMRS_SCALEDOWN);

#if (defined MOD_STBC || defined MOD_STLC)
	LOG_E(PHY, "[vTX] configured and will start : mode %i, preamble length : %i, fftSize : %i, CP: %i/%i, F_offset : %i, preamble time offset : %i, No. of subc : %i, No. of TX Ant : %i\n\n",
		cfgInfo->mode,
		cfgInfo->preamble_length,
		cfgInfo->fftSize,
		cfgInfo->nbCp,
		cfgInfo->nbCp0,
		cfgInfo->fOffset_firstCarrier,
		cfgInfo->tOffset_preamble,
		cfgInfo->nbSubcarrier,
		cfgInfo->TXAnt_nb);
#else
	LOG_E(PHY, "[vTX] configured and will start : mode %i, preamble length : %i, fftSize : %i, CP: %i/%i, F_offset : %i, preamble time offset : %i, No. of subc : %i\n\n",
		cfgInfo->mode,
		cfgInfo->preamble_length,
		cfgInfo->fftSize,
		cfgInfo->nbCp,
		cfgInfo->nbCp0,
		cfgInfo->fOffset_firstCarrier,
		cfgInfo->tOffset_preamble,
		cfgInfo->nbSubcarrier);

#endif
	
	return 0;
}



static void vtx_changeHwStatus(vtx_rtCfg_t* rtCfgInfo, txHwStatus_e status)
{
	rtCfgInfo->status = status;
	AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_outHal), "");
	vtx_HALix_out->hwStatus = status;
	AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_outHal), "");
}


static int vtx_checkInputData(vtx_rtCfg_t* rtCfgInfo)
{
	AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_inHal), "error on TX HAL MUTEX while reading RT cfg");
	rtCfgInfo->length[0] = vtx_HALix_in->length[0];
	AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_inHal), "error on TX HAL MUTEX while reading RT cfg");
	
	//if data exist,
	if (rtCfgInfo->length[0] > 0)
	{
		AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_inHal), "error on TX HAL MUTEX while reading RT cfg");
		rtCfgInfo->txData[0] = vtx_HALix_in->txData[0];
		AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_inHal), "error on TX HAL MUTEX while reading RT cfg");
		
		return 1;
	}

	return 0;
}

//allocate the data symbol into the time-frequency resource
//dPtr : data pointer
//remainedLen : total data size (remained)
//slot_nr, symb_nr : slot/OFDM symbol number
static int vtx_allocateSymbol(vtx_cfgInfo_t* cfgInfo, 
#if (defined MOD_STBC || defined MOD_STLC)
									int AntID, 
#endif
									int* dPtr, int remainLen, int slot_nr, int symb_nr)
{
	int prb_nr = 0;
	int k = cfgInfo->fOffset_firstCarrier;
	int allocatedLen = 0;

	while (remainLen - allocatedLen > 0 && allocatedLen < cfgInfo->nbSubcarrier)
	{
		int cpyLen = remainLen - allocatedLen;
		
		if (cpyLen > 12)
			cpyLen = 12;

		//allocate a PRB
#if (defined MOD_STBC || defined MOD_STLC)
		memcpy(&vtx_freqBuffer[AntID][slot_nr][symb_nr][k], dPtr, cpyLen*sizeof(int));
#else
		memcpy(&vtx_freqBuffer[slot_nr][symb_nr][k], dPtr, cpyLen*sizeof(int));
#endif
		allocatedLen += cpyLen;

		//allocate DMRS
#if (defined MOD_STBC || defined MOD_STLC)
		vtx_insertDmrs(cfgInfo, AntID, slot_nr, prb_nr, symb_nr);
#else		
		vtx_insertDmrs(cfgInfo, slot_nr, prb_nr, symb_nr);
#endif

		//data pointer shift
		dPtr = dPtr + cpyLen;

		//subcarrier offset shift
		k = (k + 12)%cfgInfo->fftSize;
		prb_nr++;
	}

	return allocatedLen;
}

#ifdef MOD_STBC
#define VTX_ANT0_SYM0_INDEX 	0
#define VTX_ANT0_SYM1_INDEX 	1
#define VTX_ANT1_SYM0_INDEX 	2
#define VTX_ANT1_SYM1_INDEX 	3


static void vtx_clearPrecodeBuf(void)
{	
	for(int i=0;i<2*HW_NB_TXANT;i++)
		memset(&vtx_precodeBuffer[i][0], 0, VHW_NB_MAXFFTSIZE*sizeof(int));
}

static void vtx_STBC_Precoder(int len, short* dPtr)
{
	short* output;
	for(int i=0; i<len/2 ; i++)
	{
		output = (short*)(&vtx_precodeBuffer[VTX_ANT0_SYM0_INDEX][i]);
		output[0] = dPtr[i*4];
		output[1] = dPtr[i*4+1];

		output = (short*)(&vtx_precodeBuffer[VTX_ANT0_SYM1_INDEX][i]);
		output[0] = - dPtr[i*4+2];
		output[1] = dPtr[i*4+3];

		output = (short*)(&vtx_precodeBuffer[VTX_ANT1_SYM0_INDEX][i]);
		output[0] = dPtr[i*4+2];
		output[1] = dPtr[i*4+3];

		output = (short*)(&vtx_precodeBuffer[VTX_ANT1_SYM1_INDEX][i]);
		output[0] = dPtr[i*4];
		output[1] = - dPtr[i*4+1];		
	}

}
#endif

//write to TX frequency buffer Bp first
static void vtx_readBuffer(vtx_cfgInfo_t* cfgInfo, vtx_rtCfg_t* rtCfgInfo)
{
	uint32_t len, remainLen;
	int *dPtr;
	uint8_t symOffset;
	

#if (defined MOD_STBC || defined MOD_STLC)
	int symbBlockSize=1;
	int accum_allocated=0;
	int symbIndex=0;

	if (cfgInfo->mode == hw_trxMode_1x2STLC || cfgInfo->mode == hw_trxMode_2x1STBC)
	{
		symbBlockSize = 2;
	}
#endif

	if (rtCfgInfo->slotSize < rtCfgInfo->length[0])
		len = rtCfgInfo->slotSize;
	else
		len = rtCfgInfo->length[0];

	if (len > 0)
	{
#if (defined MOD_STBC || defined MOD_STLC)
		remainLen = len/symbBlockSize;
#else
		remainLen = len;
#endif
		symOffset = 0;
		dPtr = rtCfgInfo->txData[0];

#if (defined MOD_STBC)
		vtx_clearPrecodeBuf();
		if (cfgInfo->mode == hw_trxMode_2x1STBC)
		{
			//STBC precoding
			vtx_STBC_Precoder(rtCfgInfo->length[0], (short*)rtCfgInfo->txData[0]);
		}
#endif

		while (remainLen > 0 && symOffset < HW_NB_SYM_IN_SLOT)
		{
			if ( (rtCfgInfo->slot_nr_sch == VHW_PREAMBLE_SLOT && symOffset == VHW_PREAMBLE_SYMBOL) || //preamble OFDM symbol : skip it
				(vtrx_checkDmrsSymb(symOffset) == 1)) //DMRS symbol : skip it
			{
				symOffset++;
			}
			else //allocate symbol
			{
				int allocatedLen=0;
			
#if (defined MOD_STBC || defined MOD_STLC)
				for (int i=0;i<cfgInfo->TXAnt_nb;i++)
				{
					if (cfgInfo->mode == hw_trxMode_siso)
					{
						dPtr = (int*)rtCfgInfo->txData[0] + accum_allocated;
					}
					else if (cfgInfo->mode == hw_trxMode_2x1STBC)
					{
						dPtr = (int*)&vtx_precodeBuffer[i*symbBlockSize+symbIndex][accum_allocated];	
					}
					allocatedLen = vtx_allocateSymbol(cfgInfo, i, dPtr, remainLen, rtCfgInfo->slot_nr_sch, symOffset);

					if (allocatedLen > 0)
						vtx_freqBufferBp[i][rtCfgInfo->slot_nr_sch][symOffset] = 1;
				}
#else
				allocatedLen = vtx_allocateSymbol(cfgInfo, dPtr, remainLen, rtCfgInfo->slot_nr_sch, symOffset);
				if (allocatedLen > 0)
					vtx_freqBufferBp[rtCfgInfo->slot_nr_sch][symOffset] = 1;
				dPtr += allocatedLen;
#endif					
				symOffset++;
#if (defined MOD_STBC || defined MOD_STLC)
				symbIndex = (symbIndex+1)%symbBlockSize;
				if (symbIndex == 0)
#endif					
				{
					remainLen -= allocatedLen;
#if (defined MOD_STBC || defined MOD_STLC)
					accum_allocated += allocatedLen;
#endif
				}
			}
		}
	}
}


static uint8_t vtx_readOnOff(void)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_bufHal), "error on TX HAL MUTEX while reading onOff");
	onOff = vtx_HALix_buf->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_bufHal), "error on TX HAL MUTEX while reading onOff");

	return onOff;
}




static void vtx_set_processBitmap(uint8_t procCnt)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
	vtx_rfRegPtr->sharedReg.processBitmap |= (1<<procCnt);
	AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
}


static uint8_t vtx_check_processBitmap(uint8_t procCnt)
{
	uint8_t res;
	AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
	res = (vtx_rfRegPtr->sharedReg.processBitmap & (1<<procCnt));
	AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");

	return res;
}




void *vTX_mainThread(void *arg)
{
	static int __thread UE_thread_tx_retval;
	char threadname[128];
	
	uint8_t onOffCfg;
	vtx_cfgInfo_t cfgInfo;
	vtx_rtCfg_t rtCfgInfo;
	uint8_t sendFlag = 0;
	uint16_t symBitmap = 0;
	hwIxStatus_e rfst;
	#ifdef VTX_DBGPROC	
	long long timer;
	#endif

	#ifdef VTX_DBGSLOTDUMP
	int frame_cnt = 0;
	#endif
	
	int time_offset=0;
	int freqOffset;
	int detect_offset=0;
	int Datanum = 0;
	int syncStatus=0;
	int slot_nr=0;
	int dumpCnt=0;
	
	//register initialization ------------------------------------
	vtx_HALix_in 	= &( ((ix_halTx_t*)arg)->woReg );   	//HAL -> TX In Reg, HAL write Reg
	vtx_HALix_out 	= &( ((ix_halTx_t*)arg)->roReg );		//TX -> HAL Out reg, HAL read Reg
	vtx_HALix_buf 	= &( ((ix_halTx_t*)arg)->rwReg );		//TX <-> HAL

	vtx_muPtr_outHal = (pthread_mutex_t*)&(((ix_halTx_t*)arg)->roReg.mutex_roHal);
	vtx_muPtr_inHal = (pthread_mutex_t*)&(((ix_halTx_t*)arg)->woReg.mutex_woHal);
	vtx_muPtr_bufHal = (pthread_mutex_t*)&(((ix_halTx_t*)arg)->rwReg.mutex_rwHal);	
	
	vtx_csPtr_slotIrqHal = (pthread_cond_t*)&(((ix_halTx_t*)arg)->roReg.cond_roHal);	//TX -> protocol IRQ
	vtx_csPtr_inHal = (pthread_cond_t*)&(((ix_halTx_t*)arg)->woReg.cond_woHal);			//protocol -> TX 
	vtx_csPtr_bufHal = (pthread_cond_t*)&(((ix_halTx_t*)arg)->rwReg.cond_rwHal);
	
	vtx_HALix_out->hwStatus = txst_null;

	//Buffer initialization ----------------------------------------
	//main TX buffer for specific bank
#if (defined MOD_STBC || defined MOD_STLC)
	for (int i=0;i<HW_NB_TXANT;i++) 
#endif
		for (int j=0;j<VHW_NB_SAMPLEBANK;j++) 
			for(int k=0;k<VHW_NB_MAXSAMPLES;k++)  
			{
#if (defined MOD_STBC || defined MOD_STLC)
				vtx_buffer[i][j][k] = 0;
#else
				vtx_buffer[j][k] = 0;
#endif
			}

	//thread initialization -----------------------------------------
	sprintf(threadname, "vTX");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY-1, threadname);

	//HW interface initialization -------------------------------------
	// vs. RF
	AssertFatal ( NULL != (vtx_rfRegPtr = vrf_configRfReg()), "[vHW][ERROR] error in configuration RF-TX register : pointer is NULL!\n");
	
	LOG_I(VTX, "TX virtual HW is initialized, waiting for on signal \n");
	do
	{
		usleep(100);
		AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->roReg.regMutex)), "");
		rfst = vtx_rfRegPtr->roReg.hwStatus;
		AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->roReg.regMutex)), "");
		
	} while (rfst == hwIx_null);

	LOG_I(VTX, " TX virtual HW is starting now \n");

	vtx_changeHwStatus(&rtCfgInfo, txst_off);    //first, hw status of tx = off
	vtx_HALix_out->slot_nr = 0;
	vtx_HALix_buf->sch_completed = 0;
	
	while (!NPAL_checkEnd())
	{
		//off state loop (stay until on signal comes)
		AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_bufHal), "");
		while (vtx_HALix_buf->onoff == 0)
			pthread_cond_wait( vtx_csPtr_bufHal, vtx_muPtr_bufHal );
		AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_bufHal), "");
		
		//check on/off register and process it
		//if register is 'on', then do static configuration (init and start)
		vtx_configInit(&cfgInfo, &rtCfgInfo);
		vtx_changeHwStatus(&rtCfgInfo, txst_idle);
		
		//inner loop for on operation ---------------------------------------- (1 loop for 1 slot ?)
		while ( !NPAL_checkEnd() && 
				(rtCfgInfo.status != txst_off && 
				(onOffCfg = vtx_readOnOff()) == 1) )
		{
			switch (rtCfgInfo.status)
			{
				case txst_idle:
					
					#ifdef VTX_DBGPROC						
					LOG_I(PHY, "[vTX] waiting scheduling IRQ\n"); 
					vhw_timeStamp(&timer, ">>>>>> return back to origin");
					#endif

					//wait for the slot interrupt ----------------------------------
					AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
					//when meet condition & cond_signal in -> pthread wake up
					while ( rtCfgInfo.slot_nr_sch == vtx_rfRegPtr->roReg.slot_nr%VHW_NB_SAMPLEBANK )
						// the thread waits here most of the time
					{
						pthread_cond_wait( &(vtx_rfRegPtr->sharedReg.irq_txrxInst), &(vtx_rfRegPtr->sharedReg.sharedMutex) ); 
					}
					AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");

					AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
					rtCfgInfo.slot_nr_sch = vtx_rfRegPtr->roReg.slot_nr%VHW_NB_SAMPLEBANK;
					rtCfgInfo.slotSize = vtx_rfRegPtr->roReg.slotSize;
					AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");

					#ifdef VTX_DBGPROC						
					LOG_I(PHY, "[vTX] slot IRQ from RF (%i)\n",rtCfgInfo.slot_nr_sch); 
					vhw_initTime(&timer);
					#endif

					//indicate the slot number --------------------------------------
					AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_outHal), "error on TX HAL MUTEX while writing out register");
					vtx_HALix_out->slot_nr = rtCfgInfo.slot_nr_sch;
					AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_outHal), "error on TX HAL MUTEX while writing out register");

					//interrupt to upper layer ---------------------------------
					AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_bufHal), "");
					vtx_HALix_buf->sch_completed = 0;
					pthread_cond_signal( vtx_csPtr_bufHal);
					AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_bufHal), "");

					#ifdef VTX_DBGPROC						
					vhw_timeStamp(&timer, ">>>> signal to L1");
					#endif

					//change status --------------------------------------------
					vtx_changeHwStatus(&rtCfgInfo, txst_reading);
					
					break;
					
				case txst_reading:		//tx alloacating RE ...

					AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_bufHal), "");
					while (vtx_HALix_buf->sch_completed != 1)
						pthread_cond_wait( vtx_csPtr_bufHal, vtx_muPtr_bufHal );
					vtx_HALix_buf->sch_completed = 2;
					AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_bufHal), "");

					#ifdef VTX_DBGPROC						
					vhw_timeStamp(&timer, ">>>> L1 schedule indication");
					#endif

					if (vtx_checkInputData(&rtCfgInfo) == 1)
					{
						vtx_readBuffer(&cfgInfo, &rtCfgInfo);	
					}
					vtx_changeHwStatus(&rtCfgInfo, txst_idle);

					break;
				
				
				default:  //txst_NULL
					break;
				
			}		

			if (rtCfgInfo.status == txst_idle)
			{
			
#ifdef VTX_DBGSLOTDUMP
				if (rtCfgInfo.slot_nr_sch == 0)
					frame_cnt++;
				
				if (frame_cnt > VTX_SLOTDUMPLEN)
				{
					vhw_dumpDataToFile(&vtx_rfRegPtr->roReg.rxData[1][0][0], 15360, "rxdata_slot1.txt");
					vhw_dumpDataToFile(&vtx_rfRegPtr->roReg.rxData[3][0][0], 15360, "rxdata_slot3.txt");
					AssertFatal(0,"Dump for the TX-side detected preamble slot! bye bye\n");
				}
#endif
				//Send data num = Datanum
				Datanum = (Datanum+1)%VHW_DEFAULT_FFT_SIZE; 

				if (vtx_check_processBitmap(rtCfgInfo.slot_nr_sch) == 0)
				{
#ifdef MOD_STLC
					if (rtCfgInfo.slot_nr_sch%2 == 0)
					{
#endif					
#ifdef MOD_STLC
					if(1)//after DL Sync
					{
						slot_nr = (rtCfgInfo.slot_nr_sch+3)%4 ;
						
						#ifdef VTX_DBGPROC	
						vhw_timeStamp(&timer, ">>>>>> VTX start preamble detection");
						#endif
						
						time_offset = vtx_detectPreamble(&cfgInfo, slot_nr , &freqOffset, detect_offset, syncStatus);

						#ifdef VTX_DBGPROC	
						vhw_timeStamp(&timer, ">>>>>> VTX end preamble detection");
						#endif	


						//command frequency drift :: Rx -> RF----------------
						vtx_commandFreqDrift(freqOffset);

						LOG_I(VTX, "Detection Slot nb : %i, Receive slot(UE -> BS) Time offset :%i freqOffset :%i \n", slot_nr, time_offset, freqOffset); 
						
						if(time_offset>0)
						{
							LOG_I(VTX, "Detection success \n\n"); 

							#if (defined VTX_DUMPFORCSI && defined bitsend)
							if(Datanum == 1021)
							{	
								//F-domain Dump
								//vhw_dft(&vtx_rfRegPtr->roReg.rxData[slot_nr][0][time_offset], &vrx_demodBuf[0][slot_nr][6][0], cfgInfo.fftSize);//+cfgInfo.nbCp
								//memcpy(&vrx_preambleDump[0][dumpCnt][0],&vrx_demodBuf[0][slot_nr][6][0], cfgInfo.fftSize*sizeof(int));

								//T-domain Dump
								memcpy(&vrx_Dump[0][dumpCnt][0],&vtx_rfRegPtr->roReg.rxData[slot_nr][0][3280], 6590*sizeof(int));

		
								printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Dump 1021 data(%d)\n", dumpCnt);

								dumpCnt = dumpCnt + 1;
							}

							if(Datanum != 1021 && dumpCnt == dumpSize)
							{
								//F-domain Dump
								//vhw_dumpDataToFile(&vrx_preambleDump[0][0], dumpCnt*cfgInfo.fftSize, "tx_preamble.txt");
								
								//T-domain Dump
								vhw_dumpDataToFile(&(vrx_Dump[0][0][0]), dumpCnt*6590, "tx_preamble.txt");
								AssertFatal(0,"Dump for Channel estimation by preamble! bye bye\n");
							}
							#endif		
						}
						else
						{	
							LOG_I(VTX, "Detection fail \n\n");							
						}
					}
#endif
					#ifdef VTX_DBGPROC						
					vhw_timeStamp(&timer, ">>>> VTX start modulation");
					#endif
					
#if (defined MOD_STBC || defined MOD_STLC)
					// 1 loop for 1 Ant time Sample Buf
					for(int k=0; k < cfgInfo.TXAnt_nb; k++)
#endif
					{
						symBitmap = 0;
						sendFlag = 0;
						
						// 1 loop for 1 slot (14 symbol)
						for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
						{
							if (rtCfgInfo.slot_nr_sch == VHW_PREAMBLE_SLOT && i == VHW_PREAMBLE_SYMBOL) //preamble symbol : skip it because it is already generated
							{
								sendFlag = 1;
								continue;
							}
				
							//decide which symbol to modulate
#if (defined MOD_STBC || defined MOD_STLC)
							if (vtx_freqBufferBp[k][rtCfgInfo.slot_nr_sch][i] == 1)
#else
							if (vtx_freqBufferBp[rtCfgInfo.slot_nr_sch][i] == 1)
#endif
							{
								symBitmap = symBitmap | (0x01 <<i);
								sendFlag = 1;
							}
						}
						
						if (sendFlag == 1)
						{
							//IFFT -- modulation process : modslot -> modsymbol
#if (defined MOD_STBC || defined MOD_STLC)
							vtx_modSlot(&cfgInfo, k, rtCfgInfo.slot_nr_sch, symBitmap);						

#else
							vtx_modSlot(&cfgInfo, rtCfgInfo.slot_nr_sch, symBitmap);
#endif		
			
							//Send to RF (Data) 
							AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
#if (defined MOD_STBC || defined MOD_STLC)
							memcpy(&vtx_rfRegPtr->sharedReg.txData[rtCfgInfo.slot_nr_sch][k][0], &vtx_buffer[k][rtCfgInfo.slot_nr_sch][0], rtCfgInfo.slotSize*sizeof(int));
#else
							memcpy(&vtx_rfRegPtr->sharedReg.txData[rtCfgInfo.slot_nr_sch][0][0], &vtx_buffer[rtCfgInfo.slot_nr_sch][0], rtCfgInfo.slotSize*sizeof(int));
#endif
							AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
							
							sendFlag=0;

						}					
						else  // if there is no data -> 0 padding
						{
							AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
#if (defined MOD_STBC || defined MOD_STLC)
							memset(&(vtx_rfRegPtr->sharedReg.txData[rtCfgInfo.slot_nr_sch][k][0]), 0, rtCfgInfo.slotSize*sizeof(int));
#else
							memset(&(vtx_rfRegPtr->sharedReg.txData[rtCfgInfo.slot_nr_sch][0][0]), 0, rtCfgInfo.slotSize*sizeof(int));
#endif
							AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
						}
					}					
#ifdef VTX_DBGPROC						
					vhw_timeStamp(&timer, ">>>> VTX end modulation");
#endif
					vtx_set_processBitmap(rtCfgInfo.slot_nr_sch);


#if (defined MOD_STBC || defined MOD_STLC)
					vtx_clearBuf(&cfgInfo, (0x01 << rtCfgInfo.slot_nr_sch ));  
					vtx_clearFreqBuf(&cfgInfo, (0x01 << rtCfgInfo.slot_nr_sch));  
#else
					vtx_clearBuf(&cfgInfo, (0x01 << rtCfgInfo.slot_nr_sch ));  
					vtx_clearFreqBuf(&cfgInfo, (0x01 << rtCfgInfo.slot_nr_sch));  
#endif
#ifdef MOD_STLC
					}
#endif
				}

				else
				{
					LOG_E(VTX, "[WARNING] cannot write TX buffer!! for sch slot %i (bitmap:%x), too late!\n", 
					rtCfgInfo.slot_nr_sch, vtx_rfRegPtr->sharedReg.processBitmap);
				}
			}

		}

		vtx_changeHwStatus(&rtCfgInfo, txst_off);
				
	}  // while !oai_exit

	return &UE_thread_tx_retval;
}





