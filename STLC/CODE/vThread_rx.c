#define _GNU_SOURCE


#include "vThread_rx.h"
#include "NPAL_thread.h"
#include "NPAL_engines.h"
#include "vThread_HWCommon.h"
#include "vThreadix_rf.h"
#include "vTRX_dmrs.h"

#ifdef MOD_STLC
#define VRX_DEFAULT_THRES_PREAMBLEDETECTION		10
#else
#define VRX_DEFAULT_THRES_PREAMBLEDETECTION		4
#endif

#define VRX_ONLINE_THRES_PREAMBLEDETECTION		2
#define VRX_TIMEDRIFT_THRESH					20
#define VRX_SYNC_WINDOW_SIZE					1000
#define VRX_DMRS_SLOT							2
#define VRX_MAX_OUTSYNC							10
#define VRX_THRES_FREQOFFSET					500
#define VRX_SNR_NOSYNC							-30.0
#define VRX_DMRS_AMP					(15-VTRX_DMRS_SCALEDOWN)
#define VRX_NOISE_ELEMENT_NB 					100
#define VRX_PREAMBLELEN							128

//#define VRX_DBGPRINT
//#define VRX_CORHISTDUMP
//#define VRX_CELLTRACKDUMP
//#define VRX_IMMEDIATEDUMP

#ifdef MOD_STLC
#define VHW_PREAMBLE_SLOT_UL_1		1
#define VHW_PREAMBLE_SLOT_UL_3		3
#endif

#define VRX_DUMPFORCSI
#define preambleScale				5
#define dumpSize					100

#define bitsend
//#define symbolsend
#define userxant1							// if rxant num ==2, rxant naming => rxant0, rxant1


#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef BUILDOPT_MACOS
#include <malloc.h>
#endif


#undef MAX
#define MAX(x,y) ((x)>(y)?(x):(y))
#undef MIN
#define MIN(x,y) ((x)<(y)?(x):(y))


/*    ------- BUFFER ----------------------               */
// Synch Buffer ----------------------------------------------
static short vrx_preambleSeqBuf[2*VRX_NB_MAX_PREAMBLE_SEQ] __attribute__ ((aligned (32))); //time-domain preamble sequence
static uint16_t vrx_preambleLen; //length of the time-domain preamble sequence
static uint16_t frameCnt=0;

// H Estimation Buffer ---------------------------------------
#if (defined MOD_STBC || defined MOD_STLC)
// Decoding Buffer --------------------------------------------
static int vrx_demodBuf[HW_NB_RXANT][VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));  

static int vrx_preambleBuf[HW_NB_RXANT][VHW_NB_SAMPLEBANK][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));  
static int vrx_dataBuf[HW_NB_RXANT][VHW_NB_SAMPLEBANK][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));  

//for decoding data
static short vrx_MLdataBuf[HW_NB_RXANT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vrx_decDataBuf[HW_NB_RXANT][VHW_NB_SAMPLEBANK][VHW_MAX_BLOCKSIZE]; //decoded data ----to nr_ue (final)

//calculate preamble H
static int vrx_preambleDump[HW_NB_RXANT][500*VHW_NB_SAMPLEBANK][VHW_DEFAULT_FFT_SIZE] __attribute__ ((aligned (32)));

#ifdef MOD_STLC
// Feedback Buffer
static int vtx_freqBuffer[HW_NB_RXANT][VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vtx_freqBufferBp[HW_NB_RXANT][VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT] __attribute__ ((aligned (32)));
static int vtx_buffer[HW_NB_TXANT][VHW_NB_SAMPLEBANK][VHW_NB_MAXSAMPLES] __attribute__ ((aligned (32)));
#endif

#else
// Decoding Buffer --------------------------------------------
static int vrx_demodBuf[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));  
static int vrx_decDataBuf[VHW_NB_SAMPLEBANK][VHW_MAX_BLOCKSIZE]; 	//decoded data ----to nr_ue (final)

#endif

// for check data number(=total_decimal) // dumpCnt => number of dump
int total_decimal = 0;
int dumpCnt = 0;


/* --------------- HAL configuration status type --------------- */
typedef struct { //initial configurations
	hw_trxMode_e mode;				//one shot / continuous
	int __attribute__ ((aligned (32))) preambleSeq[VRX_NB_MAX_PREAMBLE_SEQ];
	uint16_t preamble_length;
	uint16_t fftSize;
	uint16_t nbCp;					//cyclic prefix length
	uint16_t nbCp0;					//cyclic prefix length (the first carrier)
	uint16_t fOffset_firstCarrier; 	//frequency subcarrier index offset for the first subcarrier
	uint32_t tOffset_preamble;		//time offset for the preamble
	uint16_t nbSubcarrier;			//number of subcarriers
	uint32_t blockSize;				//decoding data size
#if (defined MOD_STBC || defined MOD_STLC)
	int TXAnt_nb;
	int RXAnt_nb;
#endif
} vrx_cfgInfo_t;

typedef struct { //real-time configurations
	rxHwStatus_e status;
	uint8_t decIrq;
	uint8_t slot_nr;    
	uint32_t slotSize;       
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
	vrx_preambleLen = cfgInfo->fftSize;   //1024
	int preamble[VRX_NB_MAX_PREAMBLE_SEQ] = {0};
	
	for (int i = 0; i >= 0 && i < cfgInfo->fftSize ; i++)
	{
		int preamblepoint = (vrx_preambleLen - cfgInfo->preamble_length/2 + i) % vrx_preambleLen;	// 0 ~ 63 & 960 ~ 1023		
		preamble[preamblepoint] = cfgInfo->preambleSeq[i];			
	}
	vhw_idft(preamble, (int*)vrx_preambleSeqBuf, (int)vrx_preambleLen);		
}


#ifdef MOD_STLC
int vrx_modSymbol(			uint16_t fftSize, 		//fft size
							uint16_t nbCp, 			//number of cyclic prefix
							uint16_t nbCp0, 		//number of cyclic prefix for the first OFDM symbol
							unsigned char symbol, 	//symbol number for processing IFFT
							unsigned char Ns,	  	//slot number for processing IFFT	
							int AntID				//Ant ID for MIMO
				 )
{
	unsigned int tOffset, tIfftOffset;
	int vtx_timeBuf[HW_MAX_FFTSIZE] __attribute__ ((aligned (32)));

	if (Ns<0 || Ns>=VHW_NB_SAMPLEBANK)
	{
		LOG_E(PHY, "IFFT : Ns must be between 0 and %i\n", VHW_NB_SAMPLEBANK);
		return(-1);
	}
	if (symbol<0 || symbol>=HW_NB_SYM_IN_SLOT)
	{
		LOG_E(PHY, "IFFT : Ns must be between 0 and %i\n", VHW_NB_SAMPLEBANK);
		return(-1);
	}

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
	return(0);
}


//modulation process : generate time buffer from freq buffer
//assumption : freq buffer has already generated
//Ns : slot number
//symbol bitmap : symbol to generate
int vrx_modSlot(vrx_cfgInfo_t* cfgInfo, int AntID, uint8_t Ns, uint16_t symbolBitmap)
{

	//clear the time buffer first
	//vrx_clearBuf(cfgInfo, (0x01 << Ns), AntID);

	//modulation process for each OFDM symbol in the slot Ns
	for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
	{
		if ((symbolBitmap & (0x01 << i)) > 0) // if it is given to modulation it
		{
			vrx_modSymbol(cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0,  i, Ns, AntID);
		}
	}

	return 0;
}

void vrx_clearBuf(vrx_cfgInfo_t* cfgInfo, uint8_t slotBitmap)
{
	for (int AntID=0;AntID<cfgInfo->RXAnt_nb;AntID++)
	{
		for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
		{
			if (slotBitmap & (0x01 << i))
			{
				if ( ((i != VHW_PREAMBLE_SLOT_UL_1) || (i != VHW_PREAMBLE_SLOT_UL_3)) && AntID != 0)
				{
					memset(&vtx_buffer[AntID][i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int));		//TX Ant time buffer
				}
				else //clear only the time sample part that is not for preamble OFDM symbol
				{
					memset(&vtx_buffer[AntID][i][0], 0, (cfgInfo->tOffset_preamble-1)*sizeof(int));
					memset(&vtx_buffer[AntID][i][cfgInfo->tOffset_preamble+cfgInfo->nbCp+cfgInfo->fftSize], 0, (VHW_NB_MAXSAMPLES-cfgInfo->tOffset_preamble-cfgInfo->nbCp-cfgInfo->fftSize)*sizeof(int));
				}
			}
		}
	}

	
}


void vrx_insertPreamble(vrx_cfgInfo_t* cfgInfo, vrx_rtCfg_t* rtCfgInfo)
{
	uint16_t symbol_bitmap = (0x01 << VHW_PREAMBLE_SYMBOL);
	int k = cfgInfo->fftSize - cfgInfo->preamble_length/2;

	for (int i=0;i<cfgInfo->preamble_length;i++)
	{		
		vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][k] = cfgInfo->preambleSeq[i];
		vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][k] = cfgInfo->preambleSeq[i];
		vtx_freqBufferBp[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL] = 1;
		vtx_freqBufferBp[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL] = 1;

		k = (k+1)%cfgInfo->fftSize;   
	}

	#ifdef preambleScale
	for (int i=0;i<cfgInfo->fftSize;i++)
	{
		if (preambleScale < 0)
		{
			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[0] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[0]<<(-preambleScale));
			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[1] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[1]<<(-preambleScale));

			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[0] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[0]<<(-preambleScale));
			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[1] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[1]<<(-preambleScale));
		}
		else
		{
		    ((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[0] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[0]>>preambleScale);
			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[1] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_1][VHW_PREAMBLE_SYMBOL][i]))[1]>>preambleScale);

			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[0] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[0]>>preambleScale);
			((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[1] = (((short*)(&vtx_freqBuffer[0][VHW_PREAMBLE_SLOT_UL_3][VHW_PREAMBLE_SYMBOL][i]))[1]>>preambleScale);
		}
	}
	#endif

	//clear the time buffer first
	vrx_clearBuf(cfgInfo, (0x01 << VHW_PREAMBLE_SLOT_UL_1)); //make vtx_clearBuf 
	vrx_clearBuf(cfgInfo, (0x01 << VHW_PREAMBLE_SLOT_UL_3));

	//RX ANT 0 only have preamble, ANT2 preamble x
	
	vrx_modSlot(cfgInfo, 0, VHW_PREAMBLE_SLOT_UL_1, symbol_bitmap);
	vrx_modSlot(cfgInfo, 0, VHW_PREAMBLE_SLOT_UL_3, symbol_bitmap);
	
}



#if 0
//insert the generated DMRS sequence to the frequency buffer
//slot nr : slot number
//prb nr : physical RB number (1RB : 12 subcarriers)
//dmrs nr : DMRS symbol number (in time)
void vrx_insertDmrs(vrx_cfgInfo_t* cfgInfo, 
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
#endif
#endif

int8_t vrx_configInit(vrx_cfgInfo_t* cfgInfo, vrx_rtCfg_t* rtCfgInfo)
{

	cfgInfo->mode = vrx_HALix_in->mode;
	cfgInfo->preamble_length = vrx_HALix_in->preamble_length; //128 
	memcpy(cfgInfo->preambleSeq, vrx_HALix_in->preambleSeq, cfgInfo->preamble_length*sizeof(int));	//hal_preambleseq[0]...
	cfgInfo->fftSize = vrx_HALix_in->fftSize;
	cfgInfo->nbCp = ((int)VHW_DEFAULT_CPLENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->nbCp0 = ((int)VHW_DEFAULT_CP0LENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->fOffset_firstCarrier = vhw_calcFirstCarrier(cfgInfo->fftSize);
	cfgInfo->tOffset_preamble = (cfgInfo->fftSize+cfgInfo->nbCp)*VHW_PREAMBLE_SYMBOL + cfgInfo->nbCp0;
	cfgInfo->nbSubcarrier = vhw_calcNbSubcarrier(cfgInfo->fftSize);
	cfgInfo->blockSize = vrx_HALix_in->decBlockSize;

	uint8_t sendFlag = 0;
	uint16_t symBitmap = 0;
	
#if (defined MOD_STBC || defined MOD_STLC)

	switch(cfgInfo->mode)
	{
		case hw_trxMode_2x1STBC:
			cfgInfo->TXAnt_nb = 2;
			cfgInfo->RXAnt_nb = 1;
			break;
			
		case hw_trxMode_1x2STLC:
			cfgInfo->TXAnt_nb = 1;
			cfgInfo->RXAnt_nb = 2;
			break;
			
		case hw_trxMode_siso:			
		default:
			cfgInfo->TXAnt_nb = 1;
			cfgInfo->RXAnt_nb = 1;
			break;			
	}
#endif

	//preamble calculation
	vrx_genPreamble(cfgInfo);

	//generate pilot sequence
	vtrx_genDmrs(cfgInfo->nbSubcarrier,
#if (defined MOD_STBC || defined MOD_STLC)
					cfgInfo->mode,
#endif
					0);

//generate feedback buffer(vtx_buffer)
#ifdef MOD_STLC
	//insert preamble
	vrx_insertPreamble(cfgInfo, rtCfgInfo);

	//insert DMRS
	#if 0
	vrx_insertDmrs(cfgInfo, rtCfgInfo->AntID, rtCfgInfo->slot_nr, rtCfgInfo->prb_nr, rtCfgInfo->symb_nr);
	#endif

	#if 0
	for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
	{
		sendFlag = 0;
		if (rtCfgInfo->slot_nr == VHW_PREAMBLE_UESLOT_STLC && i == VHW_PREAMBLE_SYMBOL) //preamble symbol : skip it because it is already generated
		{
			sendFlag = 1;
			continue;
		}
		if (vtx_freqBufferBp[k][rtCfgInfo.slot_nr_sch][i] == 1)
		{
			symBitmap = symBitmap | (0x01 <<i);
			sendFlag = 1;
		}
	}


	if (sendFlag == 1)
	{
		vrx_modSlot(cfgInfo, rtCfgInfo->AntID, rtCfgInfo->slot_nr, symBitmap);
	}
	#endif	
	
	#if 0
		//Send to RF
		AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
		rtCfgInfo->slot_nr = (vrx_rfRegPtr->roReg.slot_nr)%VHW_NB_SAMPLEBANK;
		rtCfgInfo->slotSize = vrx_rfRegPtr->roReg.slotSize;
		AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	#endif

	#if 0
	for (int k=0;k<cfgInfo->RXAnt_nb;k++)
	{
		AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
			memcpy(&vrx_rfRegPtr->sharedReg.txData[rtCfgInfo->slot_nr][k][0], &vtx_buffer[cfgInfo->RXAnt_ID[k]][rtCfgInfo->slot_nr][0], rtCfgInfo->slotSize*sizeof(int));
		AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	}
	#endif
#endif


#if (defined MOD_STBC || defined MOD_STLC)
	LOG_E(PHY, "[vRX] configured and will start : mode %i, preamble length : %i, FFT size : %i, CP:%i/%i, preamble offset:%i, block Size : %i, TX/RX ant : %i/%i\n",
		cfgInfo->mode, cfgInfo->preamble_length, cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0, cfgInfo->tOffset_preamble, cfgInfo->blockSize, cfgInfo->TXAnt_nb, cfgInfo->RXAnt_nb);
#else
	LOG_E(PHY, "[vRX] configured and will start : mode %i, preamble length : %i, FFT size : %i, CP:%i/%i, preamble offset:%i, block Size : %i\n",
		cfgInfo->mode, cfgInfo->preamble_length, cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0, cfgInfo->tOffset_preamble, cfgInfo->blockSize);
#endif

	vrx_HALix_buf->decIrq = 0;
	rtCfgInfo->decIrq = 0;

	return 0;
}


static void vrx_configRealTime(vrx_rtCfg_t* rtCfgInfo)
{

}


//detect preamble-rxdata freqoffset - Coarse estimation
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

	//estimation of fractional frequency offset: angle[(result1)'*(result2)]/pi
	return (atan2(re1*im2-re2*im1,re1*re2+im1*im2)/VHW_PI);
}

#define CORR_DOWNSCALE		10

static int vrx_detectPreamble (vrx_cfgInfo_t* cfgInfo, 
							   uint8_t proc_nr, 
							   int *freqOffset, 	
							   int detect_offset) 
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->roReg.regMutex)), "");
	int slotSize = vrx_rfRegPtr->roReg.slotSize;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->roReg.regMutex)), "");
	
	int start = 0;
	int end = slotSize;
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

	//amplitude estimation -------------------------------------------------	
	ptr16 = &(vrx_preambleSeqBuf[0]);
	for (int i=0; i<2*vrx_preambleLen; i++)
	{
		maxval = max(maxval,ptr16[i]);
		maxval = max(maxval,-ptr16[i]);
	}
	shift = vhw_log2Approx(maxval);

	//window search command
	if (detect_offset > 0)
	{
		start = detect_offset - VRX_SYNC_WINDOW_SIZE/2;
		end = detect_offset + VRX_SYNC_WINDOW_SIZE/2;
	}
	
	//-----------------------------------------------------------------------
	for (int n=start; n < end; n+=4) // n= 0 ~ end-4 (slotSize)
	{ 
		if ( n < (slotSize - vrx_preambleLen) )
		{
			int64_t dotResult	= dot_product64(&(vrx_preambleSeqBuf[0]),
								(short*) &(vrx_rfRegPtr->roReg.rxData[proc_nr][0][n]), 
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

	LOG_D(VRX, "[detection result] max_offset : %i \n" , max_offset);		
	

	if ((detect_offset == 0 && max_result < VRX_DEFAULT_THRES_PREAMBLEDETECTION*av_result) ||
		(detect_offset > 0 && max_result < VRX_ONLINE_THRES_PREAMBLEDETECTION*av_result))
	{
		LOG_D(VRX, "peak-to-average is below threshold! (max:%li av:%li) (start:%i, end:%i) end with assertion!\n", max_result, av_result, start, end);
		return(-1);
	}

	//f_off estmation :: angle, digital freq -----------------------------------
	f_off = vrx_estFreqOffsetFromPss(&(vrx_preambleSeqBuf[0]), (short*) &(vrx_rfRegPtr->roReg.rxData[proc_nr][0][max_offset]), vrx_preambleLen, shift);	
	
	//freqOffset = analog freq
	*freqOffset = (int)(f_off*(15360000.0/vrx_preambleLen));

	if (start == 0) // if it is full search
		LOG_I(PHY, "[VRX] Found preamble - offset : %i, max result : %li, av_result : %li, frequency offset : %lf (%lf)\n", 
								max_offset, max_result, av_result, f_off, f_off*(15360000.0/vrx_preambleLen));

	if (*freqOffset > 300 || *freqOffset < -300)
		LOG_E(PHY, "[VRX] WARNING ::: too large frequency offset : %i, needs to be compensated immediately\n", *freqOffset);

	
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


#if 0
static void vrx_forceOff(void)
{
	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_bufHal), "error on RX HAL MUTEX while forcing onOff");
	vrx_HALix_buf->onoff = 0;
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_bufHal), "error on RX HAL MUTEX while forcing onOff");
}

#endif

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

#if 0
static void vrx_clear_processBitmapOn(void)
{
	vrx_rfRegPtr->sharedReg.processBitmap &= ~VRF_PROCBITMAP_MASK_ONOFF;
}
#endif

static int vrx_increment_slotCnt(int cnt)
{
	cnt = cnt + 1;
	return (cnt%VHW_NB_SAMPLEBANK);
}

#if (defined MOD_STBC || defined MOD_STLC)
static void vrx_setData(int* dataPtr, int size, int slot_nr, float* snr, int ant_nb)
#else
static void vrx_setData(int* dataPtr, int size, int slot_nr, float snr)
#endif
{
	AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_outHal), "");
	memcpy(&(vrx_HALix_out->data[slot_nr][0]), dataPtr, size*sizeof(int));
#if (defined MOD_STBC || defined MOD_STLC)
	for (int i=0;i<ant_nb;i++)
		vrx_HALix_out->snr[i] = snr[i];
#else
	vrx_HALix_out->snr[0] = snr;
#endif
	AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");
}


//AntID : symIndex*HW_NB_RXANT*HW_NB_TXANT + TXID*HW_NB_RXANT + RXID

#if (defined MOD_STBC || defined MOD_STLC)
static int vrx_decodeSymbol(vrx_cfgInfo_t *cfgInfo, int linkID, int slot_nr, int decLen, int symOffset, int dmrs_nr, int dmrs_symb_nr, int decOffset)
#else
static int vrx_decodeSymbol(vrx_cfgInfo_t *cfgInfo, int slot_nr, int decLen, int symOffset, int dmrs_nr, int dmrs_symb_nr, int decOffset)
#endif
{
	int k = cfgInfo->fOffset_firstCarrier;
	int decCnt=0;

#if (defined MOD_STBC || defined MOD_STLC)	
	uint8_t txAntID = (linkID%(HW_NB_TXANT*HW_NB_RXANT))/HW_NB_RXANT;
	uint8_t rxAntID = linkID%HW_NB_RXANT;
	uint8_t symIndex = linkID/(HW_NB_TXANT*HW_NB_RXANT);
#endif

#ifdef userxant1
	short exphalfpi1[2] = {0, -1};
	short exphalfpi2[2] = {0, 1};
#endif

	for (int i=0; i<decLen/2; i++)
	{
		int chEst;
		int shift;
		int pilot = vtrx_getDmrsSymb(slot_nr, dmrs_nr, i);
		short *ptrPil=(short*)&pilot;

		//demapping (ith dmrs symbol)
#if (defined MOD_STBC || defined MOD_STLC)
		short *ptrRx = (short*)&vrx_demodBuf[rxAntID][slot_nr][dmrs_symb_nr][k];
		short *ptrData = (short*)&vrx_demodBuf[rxAntID][slot_nr][symOffset][k];
#else
		short *ptrRx = (short*)&vrx_demodBuf[slot_nr][dmrs_symb_nr][k];
		short *ptrData = (short*)&vrx_demodBuf[slot_nr][symOffset][k];
#endif
#if (defined MOD_STBC || defined MOD_STLC)
		switch(cfgInfo->mode)
		{
			case hw_trxMode_2x1STBC:
				
				if (symIndex == 0)//odd symbol
				{
					chEst = vhw_calcConjMult(ptrPil, ptrRx, VRX_DMRS_AMP);
					shift = vhw_calcAmplitude((short*)&chEst);
					short *chEstPtr = (short*)&chEst;
					LOG_D(VRX, "channel estimation odd (link:%i (tx%i/rx%i/t%i) slot %i, sym:%i, dmrs_NR:%i, dmrs_symb_nr : %i, k %i) pilot : (%i %i), prx : (%i, %i) - H:(%i %i), data : (%i, %i), shift : %i\n",
									linkID, txAntID, rxAntID, symIndex, slot_nr, symOffset, dmrs_nr, dmrs_symb_nr, k, ptrPil[0], ptrPil[1], ptrRx[0], ptrRx[1], chEstPtr[0], chEstPtr[1], ptrData[0], ptrData[1], shift);

					//channel compensation
					vrx_decDataBuf[rxAntID][slot_nr][decOffset+2*i+txAntID] = vhw_calcConjMult((short*)&chEst, ptrData, shift);
				}
				else //even symbol
				{
					chEst = vhw_calcConjMult(ptrPil, ptrRx, VRX_DMRS_AMP);
					shift = vhw_calcAmplitude((short*)&chEst);
					short *chEstPtr = (short*)&chEst;
					LOG_D(VRX, "channel estimation even (link:%i (tx%i/rx%i/t%i) slot %i, sym:%i, dmrs_NR:%i, dmrs_symb_nr : %i, k %i) pilot : (%i %i), rx : (%i, %i) - H:(%i %i), data : (%i, %i), shift : %i\n",
									linkID, txAntID, rxAntID, symIndex, slot_nr, symOffset, dmrs_nr, dmrs_symb_nr, k, ptrPil[0], ptrPil[1], ptrRx[0], ptrRx[1], chEstPtr[0], chEstPtr[1], ptrData[0], ptrData[1], shift);

					//channel compensation
					if (txAntID == 0)
						vrx_decDataBuf[rxAntID][slot_nr][decOffset+2*i+(txAntID+1)%2] -= vhw_calcConjMult(ptrData, (short*)&chEst, shift);						
					else
						vrx_decDataBuf[rxAntID][slot_nr][decOffset+2*i+(txAntID+1)%2] += vhw_calcConjMult(ptrData, (short*)&chEst, shift);						
					
					decCnt++;
				}
				
				break;
				
			case hw_trxMode_1x2STLC:
				// shift scaling check
				// if wireless channel, 
				// Ant 0 => chEst scaling = 8, vrx_decDataBuf scaling = 11 (with no attenuator) 
				// Ant 1 => chEst scaling = 10, vrx_decDataBuf scaling = 13 (with no attenuator)
				// else if wire channel, check num according to attenuator scale
				// ex) 20dB => Ant0 chEst scaling = 11, vrx_decDataBuf scaling =14
				
				//channel estimation
				if (rxAntID == 0)
					chEst = vhw_calcConjMult(ptrPil, ptrRx, 11);
				else if(rxAntID == 1)
					chEst = vhw_calcConjMult(ptrPil, ptrRx, 11);
				
				//chEst = vhw_calcConjMult(ptrPil, ptrRx, VRX_DMRS_AMP);
				shift = vhw_calcAmplitude((short*)&chEst);
				
				if (symIndex == 0)
				{
					short *chEstPtr = (short*)&chEst;
					LOG_D(VRX, "channel estimation (link:%i (tx%i/rx%i/t%i) (slot %i, sym:%i, dmrs_NR:%i, dmrs_symb_nr : %i, k %i) pilot : (%i %i), rx : (%i, %i) - H:(%i %i), data : (%i, %i), shift : %i\n",
												linkID, txAntID, rxAntID, symIndex, slot_nr, symOffset, dmrs_nr, dmrs_symb_nr, k, ptrPil[0], ptrPil[1], ptrRx[0], ptrRx[1], chEstPtr[0], chEstPtr[1], ptrData[0], ptrData[1], shift);

					//channel compensation
					if (rxAntID == 0)
						vrx_decDataBuf[rxAntID][slot_nr][decOffset+i] = vhw_calcConjMult((short*)&chEst, ptrData, 14);
					else if(rxAntID == 1)
						vrx_decDataBuf[rxAntID][slot_nr][decOffset+i] = vhw_calcConjMult((short*)&chEst, ptrData, 14);

					// if you use wireless channel, change if 0 => 1
					#if 0
						#ifdef userxant1
							if (rxAntID == 1)
								vrx_decDataBuf[rxAntID][slot_nr][decOffset+i] = vhw_calcMult((short*)&vrx_decDataBuf[rxAntID][slot_nr][decOffset+i],exphalfpi1,0);
							else
								vrx_decDataBuf[rxAntID][slot_nr][decOffset+i] = vhw_calcMult((short*)&vrx_decDataBuf[rxAntID][slot_nr][decOffset+i],exphalfpi2,0);
						#endif
					#endif
					
					decCnt++;
				}
				
				break;
				
			case hw_trxMode_siso:
			default:
#else
				//channel estimation
				chEst = vhw_calcConjMult(ptrPil, ptrRx, VRX_DMRS_AMP);
				shift = vhw_calcAmplitude((short*)&chEst);			

				short *chEstPtr = (short*)&chEst;
				LOG_D(VRX, "channel estimation (slot %i, sym:%i, dmrs_NR:%i, dmrs_symb_nr : %i, k %i) pilot : (%i %i), rx : (%i, %i) - H:(%i %i), shift : %i\n",
								slot_nr, symOffset, dmrs_nr, dmrs_symb_nr, k, ptrPil[0], ptrPil[1], ptrRx[0], ptrRx[1], chEstPtr[0], chEstPtr[1], shift);

				//channel compensation
				vrx_decDataBuf[slot_nr][decOffset+i] = vhw_calcConjMult((short*)&chEst, ptrData, shift);
				decCnt++;
#endif


#if (defined MOD_STBC || defined MOD_STLC)				
				break;
		}
#endif
		//subcarrier offset re-calculation
		k = (k+1)%cfgInfo->fftSize;
	}
	
	return decCnt;
}


static void vrx_decodeData(vrx_cfgInfo_t *cfgInfo, uint8_t slot_nr, uint8_t proc_nr)
{
	int symOffset = 0;
	int remainLen = cfgInfo->blockSize;
	int dmrsDemodulated[HW_NB_SYM_IN_SLOT];
	int decOffset = 0;
#if (defined MOD_STBC || defined MOD_STLC)
	int symbBlockSize=1;
#endif
	int symIndex=0;

	for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
	{
		dmrsDemodulated[i] = 0;
	} 

#if (defined MOD_STBC)
	if (cfgInfo->mode == hw_trxMode_2x1STBC)
	{
		symbBlockSize = 2;
	}
	remainLen = remainLen/symbBlockSize;
#endif

#ifdef MOD_STLC	
	if (cfgInfo->mode == hw_trxMode_1x2STLC)
	{
		symbBlockSize = 1;
	}
	remainLen = remainLen/symbBlockSize;

#endif

	//for each OFDM symbol
	while (symOffset < HW_NB_SYM_IN_SLOT && remainLen > 0)
	{
#ifdef MOD_STLC
		int sampleOffset = cfgInfo->nbCp0 + cfgInfo->nbCp + symOffset*(cfgInfo->fftSize + cfgInfo->nbCp);
#else
		int sampleOffset = cfgInfo->nbCp0 + cfgInfo->nbCp/2 + symOffset*(cfgInfo->fftSize + cfgInfo->nbCp);
#endif
		
		int decLen = cfgInfo->nbSubcarrier;
		
		int dmrs_nr, dmrs_symb_nr;
		int sampleOffset_dmrs;
		int decCnt=0;

		if (decLen > remainLen)
			decLen = remainLen;

		
#if (defined MOD_STBC || defined MOD_STLC)
		for (int i=0;i<cfgInfo->RXAnt_nb;i++)
#endif
		{
			//  FFT
#if (defined MOD_STBC || defined MOD_STLC)
			vhw_dft(&vrx_rfRegPtr->roReg.rxData[proc_nr][i][sampleOffset], &vrx_demodBuf[i][slot_nr][symOffset][0], cfgInfo->fftSize );
#else
			vhw_dft(&vrx_rfRegPtr->roReg.rxData[proc_nr][0][sampleOffset], &vrx_demodBuf[slot_nr][symOffset][0], cfgInfo->fftSize );
#endif

#if (defined MOD_STBC || defined MOD_STLC)
			// DMRS demodulation
			for (int k=0;k<cfgInfo->TXAnt_nb;k++)
#endif				
			{
				uint8_t linkID;
#if (defined MOD_STBC || defined MOD_STLC)
				dmrs_nr = vtrx_getChestDmrsNb(symOffset, k);
#else
				dmrs_nr = vtrx_getChestDmrsNb(symOffset);
#endif
				dmrs_symb_nr = vtrx_getChestSymbNb(dmrs_nr);
				if ( dmrsDemodulated[dmrs_symb_nr] == 0 )
				{
#ifdef MOD_STLC
					sampleOffset_dmrs = cfgInfo->nbCp0 + cfgInfo->nbCp + dmrs_symb_nr*(cfgInfo->fftSize + cfgInfo->nbCp);
#else
					sampleOffset_dmrs = cfgInfo->nbCp0 + cfgInfo->nbCp/2 + dmrs_symb_nr*(cfgInfo->fftSize + cfgInfo->nbCp);
#endif
					
#if (defined MOD_STBC || defined MOD_STLC)
					for (int i=0;i<cfgInfo->RXAnt_nb;i++)
						vhw_dft(&vrx_rfRegPtr->roReg.rxData[proc_nr][i][sampleOffset_dmrs], &vrx_demodBuf[i][slot_nr][dmrs_symb_nr][0], cfgInfo->fftSize );
#else			
					vhw_dft(&vrx_rfRegPtr->roReg.rxData[proc_nr][0][sampleOffset_dmrs], &vrx_demodBuf[slot_nr][dmrs_symb_nr][0], cfgInfo->fftSize );
#endif
					dmrsDemodulated[dmrs_symb_nr] = 1;
				}
							//    demapping
				//    channel compensation
#if (defined MOD_STBC || defined MOD_STLC)
				linkID = symIndex*HW_NB_TXANT*HW_NB_RXANT + k*HW_NB_RXANT+i;
				decCnt += vrx_decodeSymbol(cfgInfo, linkID, slot_nr, decLen, symOffset, dmrs_nr, dmrs_symb_nr, decOffset);
#else
				decCnt += vrx_decodeSymbol(cfgInfo, slot_nr, decLen, symOffset, dmrs_nr, dmrs_symb_nr, decOffset);
#endif	

				
			}

		}

		decOffset += decCnt;
		remainLen -= decCnt;
		
		symOffset++;

#if (defined MOD_STBC || defined MOD_STLC)
		symIndex = (symIndex+1)%symbBlockSize;
#endif

	}
	
}


#ifdef MOD_STLC
#define MLDET_SCALEDOWN 4

//void vrx_MLdetector(vrx_cfgInfo_t *cfgInfo, short* vrx_decDataBuf[HW_NB_RXANT][VHW_NB_SAMPLEBANK][VHW_MAX_BLOCKSIZE], short *vrx_MLdataBuf[HW_NB_RXANT][VHW_NB_MAXFFTSIZE], uint8_t slot_nr, uint8_t rxAnt)
void vrx_MLdetector(vrx_cfgInfo_t *cfgInfo, short* decDataBuf, short *MLdataBuf)
{
	short symbol[4][2] = {{5792, 5792}, {5792, -5792}, {-5792, 5792}, {-5792, -5792}};

	for (int i=0; i<cfgInfo->blockSize/2;i++)
	{
		int max_corr=-10000, corr_result;
		int max_index=-1;

		//printf("vrx_decDataBuf:  %d, %d   ",decDataBuf[2*i],decDataBuf[2*i+1]);
	
		for (int k=0; k<4; k++)
		{
			corr_result = ((int)(decDataBuf[2*i]))*((symbol[k][0]>>MLDET_SCALEDOWN)) + ((int)(decDataBuf[2*i+1])>>MLDET_SCALEDOWN)*((symbol[k][1])>>MLDET_SCALEDOWN);
		
			if (max_corr < corr_result)
			{
				max_corr = corr_result;
				max_index = k;
			}
		}

#ifdef symbolsend
		MLdataBuf[2*i] = symbol[max_index][0];
		MLdataBuf[2*i+1] = symbol[max_index][1];
#endif

#ifdef bitsend
		if(symbol[max_index][0] == 5792)
		{
			MLdataBuf[2*i] = 1;
		}
		else
		{
			MLdataBuf[2*i] = 0;
		}
		if(symbol[max_index][1] == 5792)
		{
			MLdataBuf[2*i+1] = 1;
		}
		else
		{
			MLdataBuf[2*i+1] = 0;
		}
#endif
	}

#ifdef bitsend 
	int decimal = 1;
	total_decimal = 0;
	
	for(int position = 0; position<cfgInfo->blockSize; position++)
	{
		if(MLdataBuf[position] == 1)
		{
			if(position == cfgInfo->blockSize - 1)
			{
				total_decimal = (total_decimal + decimal)%1024;
			}
			else
			{
				 for(int count = 0; count < cfgInfo->blockSize - (position+1); count++)
				 {
	        		decimal = decimal*2;
				 }

				 total_decimal = (total_decimal + decimal)%1024;
				 decimal = 1;
			}
		}
	}

	printf(">>>>>>>>>>>>>>>>>>>>>>>>Data Number : %d \n\n", total_decimal);
#endif
}

static void vrx_Dumppreamble(vrx_cfgInfo_t *cfgInfo, uint8_t slot_nr, int max_offset)
{
	if(total_decimal == 1021)
	{	
		vhw_dft(&vrx_rfRegPtr->roReg.rxData[slot_nr][0][max_offset], &vrx_demodBuf[0][slot_nr][6][0], cfgInfo->fftSize);	
		memcpy(&vrx_preambleDump[0][dumpCnt][0], &vrx_demodBuf[0][slot_nr][6][0], cfgInfo->fftSize*sizeof(int));

		printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Dump 1021 data(%d)\n", dumpCnt);
		dumpCnt = dumpCnt + 1;
	}

	if(dumpCnt == dumpSize && total_decimal != 1021)
	{	
		//F-domain Dump
		vhw_dumpDataToFile(&vrx_preambleDump[0][0], dumpCnt*cfgInfo->fftSize, "rx_preamble.txt");
		AssertFatal(0,"Dump for Channel estimation by preamble! bye bye\n");
	}
}


static void vrx_commandSlotSync(int proc_nr, int slot_nr, int syncStatus)
{
	uint8_t res = 0;
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	vrx_rfRegPtr->sharedReg.syncStatus = syncStatus;
	
	if (syncStatus == 1)
	{
		vrx_rfRegPtr->sharedReg.slotOffset = (slot_nr-proc_nr+VHW_NB_SAMPLEBANK)%VHW_NB_SAMPLEBANK;
	}
	else
	{
		vrx_rfRegPtr->sharedReg.slotOffset = 0;
	}
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
}
#endif

static void vrx_commandSyncTime(int syncTime)
{
	uint8_t res = 0;
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	if (vrx_rfRegPtr->sharedReg.syncOffset == 0)
	{
		vrx_rfRegPtr->sharedReg.syncOffset = syncTime;
		res = 1;
	}
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");

	if (res == 0)
	{
		LOG_E(PHY, "Failed to set synch offset command!! %i\n", syncTime);
	}
}


static void vrx_commandTimeDrift(int timeDrift)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	vrx_rfRegPtr->sharedReg.timeDrift = timeDrift;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
}


static void vrx_commandFreqDrift(int freqDrift)
{
	AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
	vrx_rfRegPtr->sharedReg.freqDrift = freqDrift;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
}


static double vrx_calcSNR(short* noise, int nSize, short* signal, int sSize)
{
	int64_t noisePower=0, sigPower=0;
	for (int i=0;i<nSize;i++)
	{
		noisePower += (int64_t)((int)(noise[2*i]*noise[2*i]) + (int)(noise[2*i+1]*noise[2*i+1]));
	}

	for (int i=0;i<sSize;i++)
	{
		sigPower += (int64_t)((int)(signal[2*i]*signal[2*i]) + (int)(signal[2*i+1]*signal[2*i+1]));
	}

	LOG_D(VRX, "noise : %li, signal : %li\n", noisePower/nSize, sigPower/sSize);
	return ((double)sigPower/sSize - (double)noisePower/nSize)/(noisePower/nSize);
}


static double vrx_calcPreambleSNR(int* rxdata, vrx_cfgInfo_t* cfgInfo)
{
	int rxFdata_preamble[VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));

	vhw_dft(rxdata, rxFdata_preamble, cfgInfo->fftSize);

	return 10*log10(vrx_calcSNR((short*)(rxFdata_preamble+(cfgInfo->fftSize-cfgInfo->preamble_length/2-VRX_NOISE_ELEMENT_NB)), VRX_NOISE_ELEMENT_NB, (short*)(rxFdata_preamble+(cfgInfo->fftSize-cfgInfo->preamble_length/2)),cfgInfo->preamble_length/2));
}


#if 0
static double vrx_calcDataSNR(int* rxdata, int symNum, vrx_cfgInfo_t* cfgInfo)
{
	int rxFdata[VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));

	vhw_dft(rxdata+(cfgInfo->nbCp0+symNum*(cfgInfo->fftSize+cfgInfo->nbCp)), rxFdata, cfgInfo->fftSize);

	return 10*log10(vrx_calcSNR((short*)(rxFdata+(cfgInfo->fOffset_firstCarrier+cfgInfo->blockSize)), NOISE_ELEMENT_NB, (short*)(rxFdata+(cfgInfo->fOffset_firstCarrier)),cfgInfo->blockSize));
}
#endif



static int vrx_detFreqOffsetStatus(int freqOffset)
{
	if (freqOffset < VRX_THRES_FREQOFFSET && freqOffset > -VRX_THRES_FREQOFFSET)
	{
		return 1;
	}
	
	return 0;
}

void *vRX_mainThread(void *arg)
{
	static int __thread UE_thread_rx_retval;
	char threadname[128];

	int outSyncCnt=0;
	uint8_t onOffCfg;
	vrx_cfgInfo_t cfgInfo;
	vrx_rtCfg_t rtCfgInfo;

	uint8_t procCnt = 0, slotCnt = 0;
	hwIxStatus_e rfst;
	int time_offset=0;
	int detect_offset=0;
	uint8_t status_synch = 0;
	int freqOffset;
	float snr[2];	
		
	//register initialization ------------------------------------
	vrx_HALix_in 	= &( ((ix_halRx_t*)arg)->woReg );
	vrx_HALix_out 	= &( ((ix_halRx_t*)arg)->roReg );
	vrx_HALix_buf 	= &( ((ix_halRx_t*)arg)->rwReg );

	vrx_muPtr_outHal = (pthread_mutex_t*)&(((ix_halRx_t*)arg)->roReg.mutex_roHal);
	vrx_muPtr_inHal = (pthread_mutex_t*)&(((ix_halRx_t*)arg)->woReg.mutex_woHal);
	vrx_muPtr_bufHal = (pthread_mutex_t*)&(((ix_halRx_t*)arg)->rwReg.mutex_rwHal); 

	vrx_csPtr_bufHal = (pthread_cond_t*)&(((ix_halRx_t*)arg)->rwReg.cond_rwHal);
	
	vrx_HALix_out->hwStatus = rxst_null;

	//HW interface initialization --------------------------------------
	// vs. RF
	AssertFatal ( NULL != (vrx_rfRegPtr = vrf_configRfReg()), "[vHW][ERROR] error in configuration RF-RX register : pointer is NULL!\n");

	//thread initialization ---------------------------------------------
	sprintf(threadname, "vRX");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY-1, threadname);

	LOG_I(VRX, "RX virtual HW is initialized, waiting for on signal \n");

	do
	{
		usleep(100);
		AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->roReg.regMutex)), "");
		rfst = vrx_rfRegPtr->roReg.hwStatus;
		AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->roReg.regMutex)), "");
		
	} while(rfst == hwIx_null);

	vrx_HALix_out->hwStatus = rxst_off;
	vrx_HALix_out->length = 0;
	
	while (!NPAL_checkEnd())
	{
		//off state loop (stay until on signal comes) ---------------------------------------
		vrx_checkOnOff();
		
		//if register is 'on', then do static configuration (init and start)
		vrx_configInit(&cfgInfo, &rtCfgInfo);
		
		//turn the register value into 'on'  :: when writing (changing value), must lock mutex
		AssertFatal ( 0== pthread_mutex_lock(vrx_muPtr_outHal), "");
		vrx_HALix_out->hwStatus = rxst_on;
		AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");
		
		//indicate to RF
		vrx_set_processBitmapOn();
		status_synch = 0;
		//vhw_dumpData( vrx_preambleSeqBuf, 2*VRX_NB_MAX_PREAMBLE_SEQ, "vrx_preambleSeqBuf");

		
		//inner loop for on operation (1 loop for 1 slot unit) ------------------------------------------------------
		while ( !NPAL_checkEnd() && 
				(vrx_HALix_out->hwStatus == rxst_on &&
				(onOffCfg = vrx_readOnOff()) == 1) )
		{
#ifdef MOD_STLC
			int slotOffsetStatus;
#endif
			vrx_checkRfIrq(procCnt); //wait until interrupt comes from RF
			vrx_configRealTime(&rtCfgInfo); //read the real-time configure register and apply in real-time
			
#ifdef MOD_STLC
			AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
			slotOffsetStatus = vrx_rfRegPtr->sharedReg.slotOffset;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
#endif

			LOG_D(PHY, "RX process in slot %i\n", 
#ifdef MOD_STLC
			procCnt
#else
			slotCnt
#endif
			);
			
			//1. Synchronization (check preamble) ---------------------------------------------------------------------------
			if (
#ifdef MOD_STLC
				slotOffsetStatus == 0 && 
#endif
				(status_synch == 0 || 
				(status_synch == 1 && procCnt == VHW_PREAMBLE_SLOT)))
			{
				//Rx Detect Preamble & Get FreqOffset ---------------------------
				if ((time_offset = vrx_detectPreamble(&cfgInfo, procCnt, &freqOffset, detect_offset)) > 0 )
				{
					snr[0] = (float)vrx_calcPreambleSNR(&vrx_rfRegPtr->roReg.rxData[procCnt][0][time_offset], &cfgInfo);
					snr[1] = (float)vrx_calcPreambleSNR(&vrx_rfRegPtr->roReg.rxData[procCnt][1][time_offset], &cfgInfo);
					slotCnt = VHW_PREAMBLE_SLOT;

					#ifdef VRX_IMMEDIATEDUMP
					vhw_dumpDataToFile(&vrx_rfRegPtr->roReg.rxData[procCnt][0][0], vrx_rfRegPtr->roReg.slotSize, "rxdata.txt");
					AssertFatal(0,"Dump for the detected preamble slot! bye bye\n");
					#endif

					//state transition function (exit)
					//command time synch :: Rx -> RF ----------------------					
					int timeShift = (time_offset - cfgInfo.tOffset_preamble - cfgInfo.nbCp);
					if (abs(timeShift) > VRX_TIMEDRIFT_THRESH)
					{
						vrx_commandSyncTime(timeShift);
						if (timeShift < 0)		//exeptional case
						{
							//slot cnt increment
							slotCnt++;
							slotCnt %= VHW_NB_SAMPLEBANK;
							
						}
					}
					else //command time drift :: Rx -> RF----------------
					{
						vrx_commandTimeDrift(timeShift);
					}

					//command frequency drift :: Rx -> RF----------------
					vrx_commandFreqDrift(freqOffset);	

					if (status_synch == 0)
					{
						LOG_E(PHY, "preamble SNR0 : %f\n", snr[0]);
						LOG_E(PHY, "preamble SNR1 : %f\n", snr[1]);
						frameCnt = 0;
						status_synch = 1;
#ifdef MOD_STLC
						vrx_commandSlotSync(procCnt, slotCnt, status_synch);
						for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
							vrx_clear_processBitmap(procCnt+i);
						procCnt = VHW_PREAMBLE_SLOT-1;
#endif
					}
					else
					{
						frameCnt++;
					}

				}
				else //detection failed
				{
#ifdef MOD_STLC
					if (status_synch == 1)
					{
						vrx_commandSlotSync(procCnt, procCnt, 0);
					}
#endif
					status_synch = 0;
					snr[0] = VRX_SNR_NOSYNC;
					
					outSyncCnt++;
					#ifdef VRX_OUTSYNCDUMP
					if (outSyncCnt > VRX_MAX_OUTSYNC)
					{
						vhw_dumpDataToFile(&vrx_rfRegPtr->roReg.rxData[procCnt][0][0], vrx_rfRegPtr->roReg.slotSize, "rxdata.txt");
						LOG_E(PHY, "max count has reached! bye bye\n");
						exit(-1);
					}
					#endif
				}
			}

			//2. if synch status, then decode data
			if (status_synch == 1)
			{	
#ifdef MOD_STLC
				if((procCnt %2)==0){
#endif			
				LOG_I(VRX, "Slot nb : %i , Send slot(BS -> UE), Time offset : %i freqOffset :%i \n",procCnt, time_offset, freqOffset);

				//for window search (time offset of preamble)  
				detect_offset = cfgInfo.tOffset_preamble + cfgInfo.nbCp;

				//decode Data
#ifdef MOD_STLC			
				vrx_decodeData(&cfgInfo, procCnt, procCnt);

				//for each rxant MLdetection
				//if you use wire channel, only use ANT0
				vrx_MLdetector(&cfgInfo, (short*)(&(vrx_decDataBuf[0][procCnt][0])), &(vrx_MLdataBuf[0][0]));
				//vrx_MLdetector(&cfgInfo, (short*)(&(vrx_decDataBuf[1][procCnt][0])), &(vrx_MLdataBuf[0][0]));

				#if (defined VRX_DUMPFORCSI && defined bitsend)
					vrx_Dumppreamble(&cfgInfo, procCnt, time_offset);
				#endif							
#else
				vrx_decodeData(&cfgInfo, slotCnt, procCnt);
#endif
				if (vrx_detFreqOffsetStatus(freqOffset) == 1)
				{
					//transfer to nr-ue first 10 data symbols 
#if (defined MOD_STBC || defined MOD_STLC)
#ifdef MOD_STLC
					vrx_setData(&(vrx_decDataBuf[0][procCnt][0]), cfgInfo.blockSize, procCnt, snr, cfgInfo.RXAnt_nb);					
#else
					vrx_setData(&(vrx_decDataBuf[0][slotCnt][0]), cfgInfo.blockSize, slotCnt, snr, cfgInfo.RXAnt_nb);	
#endif
#else
					vrx_setData(&(vrx_decDataBuf[slotCnt][0]), cfgInfo.blockSize, slotCnt, snr[0]);	
#endif

					//decIrq Rx -> L1 prtocol nr.ue 
#ifdef MOD_STLC
					vrx_indicate_rx(procCnt, time_offset);
#else
					vrx_indicate_rx(slotCnt, time_offset);
#endif
					usleep(1);
				}
				
				
				#ifdef VRX_CELLTRACKDUMP
#ifdef MOD_STLC
				if (procCnt == 2 && frameCnt > 20)
#else
				if (slotCnt == 2 && frameCnt > 20)
#endif
				{
					vhw_dumpDataToFile(&vrx_rfRegPtr->roReg.rxData[procCnt][0][0], vrx_rfRegPtr->roReg.slotSize, "rxdata.txt");
					AssertFatal(0,"Dump for the detected preamble slot! bye bye\n");
				}
				#endif
	
#ifdef MOD_STLC
				}
#endif	

#ifdef MOD_STLC
				else
				{
					LOG_D(VRX, "Slot nb : %i , Send slot(UE -> BS), freqOffset :%i \n",procCnt, freqOffset);
					
					for(int RXANT =0; RXANT<2; RXANT++)
					{
						//Send to RF (Data) 
						AssertFatal ( 0== pthread_mutex_lock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");				
						memcpy(&vrx_rfRegPtr->sharedReg.txData[procCnt][RXANT][0],&vtx_buffer[RXANT][procCnt][0], vrx_rfRegPtr->roReg.slotSize*sizeof(int));
						AssertFatal ( 0== pthread_mutex_unlock(&(vrx_rfRegPtr->sharedReg.sharedMutex)), "");
					}					
				}
#endif
			}
			else
			{
#ifdef MOD_STLC
				LOG_E(PHY, "LOST SYNC------- in slot %i, frame %i \n", procCnt, frameCnt);
				
#else
				LOG_E(PHY, "LOST SYNC------- in slot %i, frame %i \n", slotCnt, frameCnt);
#endif
				detect_offset = 0;
			}
			
			vrx_clear_processBitmap(procCnt);

			procCnt = vrx_increment_slotCnt(procCnt);
#ifndef MOD_STLC
			slotCnt = vrx_increment_slotCnt(slotCnt);
#endif
			
		}

		AssertFatal ( 0 == pthread_mutex_lock(vrx_muPtr_outHal), "");
		vrx_HALix_out->hwStatus = rxst_off;
		AssertFatal ( 0== pthread_mutex_unlock(vrx_muPtr_outHal), "");

	}  // while !oai_exit

	return &UE_thread_rx_retval;
}




