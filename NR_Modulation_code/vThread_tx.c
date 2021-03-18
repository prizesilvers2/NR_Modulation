#define _GNU_SOURCE
#include "vThread_tx.h"
#include "vThreadix_rf.h"
#include "nr-uesoftmodem.h"
#include "vThread_HWCommon.h"
#include "../../ARCH/COMMON/common_lib.h"



#include <sched.h>
#include <string.h>

#define VTX_MIN_SLOTDELAY		2

/* --------------- HAL configuration status type --------------- */
typedef struct { //initial configurations
	txHwMode_e mode;				// one shot mode only
	uint8_t slotDelay;
	int preambleSeq[VTX_NB_MAX_PREAMBLE_SEQ];
	uint32_t preamble_length;
	uint16_t fftSize;
	uint16_t nbCp;
	uint16_t nbCp0;
	uint16_t fOffset_firstCarrier;
	uint32_t tOffset_preamble;
	uint16_t nbSubcarrier;

} vtx_cfgInfo_t;

typedef struct { //real-time configurations
	txHwStatus_e status;
	uint32_t length[VHW_NB_SAMPLEBANK];				//center frequency
	void* txData[VHW_NB_SAMPLEBANK];					//u value
	uint8_t slot_nr_sch;
	uint32_t slotSize;
} vtx_rtCfg_t;

/*    ------- BUFFER ----------------------               */
static int vtx_buffer[VHW_NB_SAMPLEBANK][VHW_NB_MAXSAMPLES] __attribute__ ((aligned (32)));
static int vtx_freqBuffer[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vtx_dmrsSeqBuf[VHW_NB_SAMPLEBANK][VHW_DMRS_NBSYMB][VHW_NB_MAXFFTSIZE] __attribute__ ((aligned (32)));
static int vtx_freqBufferBp[VHW_NB_SAMPLEBANK][HW_NB_SYM_IN_SLOT] __attribute__ ((aligned (32)));

/*     ------- HAL - HW interface related ------          */
//registers
const halTx_woReg_t 		*vtx_HALix_in; 		//write only registers
halTx_roReg_t 				*vtx_HALix_out;		//read only registers
halTx_rwReg_t 				*vtx_HALix_buf;		//read only registers

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
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		for (int j=0;j<HW_NB_SYM_IN_SLOT;j++)
		{
			memset(&vtx_freqBuffer[i][j][0], 0, VHW_NB_MAXFFTSIZE*sizeof(int)); //reset the frequence buffer
			vtx_freqBufferBp[i][j] = 0;
		}
		memset(&vtx_buffer[i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int)); //reset the time buffer
	}
}

//clear the frequency buffers
//slot bitmap : clear the buffer (except preamble and DMRS parts)
void vtx_clearFreqBuf(vtx_cfgInfo_t *cfgInfo, uint8_t slotBitmap)
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
				memset(vtx_freqBuffer[i][k], 0, VHW_NB_MAXFFTSIZE*sizeof(int)); // reset the buffer set for slot i and symbol k
				vtx_freqBufferBp[i][k] = 0;
			}
		}
	}	
}

//clear the time buffers
//slot bitmap : clear the buffer (except preamble parts)
void vtx_clearBuf(vtx_cfgInfo_t* cfgInfo, uint8_t slotBitmap)
{
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		if (slotBitmap & (0x01 << i))
		{
			if ( i != VHW_PREAMBLE_SLOT )
				memset(&vtx_buffer[i][0], 0, VHW_NB_MAXSAMPLES*sizeof(int));
			else //clear only the time sample part that is not for preamble OFDM symbol
			{
				memset(&vtx_buffer[i][0], 0, (cfgInfo->tOffset_preamble-1)*sizeof(int));
				memset(&vtx_buffer[i][cfgInfo->tOffset_preamble+cfgInfo->nbCp+cfgInfo->fftSize], 0, (VHW_NB_MAXSAMPLES-cfgInfo->tOffset_preamble-cfgInfo->nbCp-cfgInfo->fftSize)*sizeof(int));
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
				 )
{
	unsigned int tOffset, tIfftOffset;

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
	
	//clear the output time-domain buffer first
	vhw_idft(&vtx_freqBuffer[Ns][symbol][0], &vtx_buffer[Ns][tIfftOffset], fftSize);

	//CP insertion
	if (symbol == 0)
	{
		memcpy(&vtx_buffer[Ns][tOffset], &vtx_buffer[Ns][tIfftOffset+fftSize-nbCp0], (nbCp0 + nbCp)*sizeof(int));
	}
	else
	{
		memcpy(&vtx_buffer[Ns][tOffset], &vtx_buffer[Ns][tIfftOffset+fftSize-nbCp], nbCp*sizeof(int));
	}

	return(0);
}


//modulation process : generate time buffer from freq buffer
//assumption : freq buffer has already generated
//Ns : slot number
//symbol bitmap : symbol to generate
int vtx_modSlot(vtx_cfgInfo_t* cfgInfo, uint8_t Ns, uint16_t symbolBitmap)
{
	//clear the time buffer first
	vtx_clearBuf(cfgInfo, (0x01 << Ns));

	//modulation process for each OFDM symbol in the slot Ns
	for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
	{
		if ((symbolBitmap & (0x01 << i)) > 0) // if it is given to modulation it
		{
			vtx_modSymbol(cfgInfo->fftSize, cfgInfo->nbCp, cfgInfo->nbCp0,  i, Ns);
		}
	}

	return 0;
}


//generate the DMRS sequene and store it in the sequence buffer
void vtx_genDmrs(vtx_cfgInfo_t* cfgInfo)
{
	int prb_nr = cfgInfo->nbSubcarrier;
	int dmrsSymb[VHW_DMRS_NBSYMB] = VHW_DMRS_SYMB;

	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
	{
		for (int k=0;k<VHW_DMRS_NBSYMB;k++)
		{
			vhw_genReferenceSignalSeq((short*)&vtx_dmrsSeqBuf[i][k][0], i, dmrsSymb[k], prb_nr);
		}
	}
}

//insert the generated DMRS sequence to the frequency buffer
//slot nr : slot number
//prb nr : physical RB number (1RB : 12 subcarriers)
//dmrs nr : DMRS symbol number (in time)
void vtx_insertDmrs(vtx_cfgInfo_t* cfgInfo, int slot_nr, int prb_nr, int dmrs_nr)
{
	int dmrsSymb[VHW_DMRS_NBSYMB] = VHW_DMRS_SYMB;
	int fOffset = (cfgInfo->fOffset_firstCarrier + 12*prb_nr)%cfgInfo->fftSize;
	
	for (int i=0;i<12;i++)
	{
		vtx_freqBuffer[slot_nr][dmrsSymb[dmrs_nr]][fOffset+i] = vtx_dmrsSeqBuf[slot_nr][dmrs_nr][12*prb_nr+i];
	}

	vtx_freqBufferBp[slot_nr][dmrsSymb[dmrs_nr]] = 1;	//mark as the frequency buffer is filled out
}

void vtx_insertPreamble(vtx_cfgInfo_t* cfgInfo)
{
	uint16_t symbol_bitmap = (0x01 << VHW_PREAMBLE_SYMBOL);
	int k = cfgInfo->fftSize - cfgInfo->preamble_length/2;
	
	for (int i=0;i<cfgInfo->preamble_length;i++)
	{
		vtx_freqBuffer[VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL][k] = cfgInfo->preambleSeq[i];
		k = (k+1)%cfgInfo->fftSize;	
		vtx_freqBufferBp[VHW_PREAMBLE_SLOT][VHW_PREAMBLE_SYMBOL] = 1;
	}
	vtx_modSlot(cfgInfo, VHW_PREAMBLE_SLOT, symbol_bitmap);
}



int8_t vtx_configInit(vtx_cfgInfo_t* cfgInfo, vtx_rtCfg_t* rtCfgInfo)
{

	cfgInfo->mode = vtx_HALix_in->mode;
	cfgInfo->slotDelay = vtx_HALix_in->slotDelay;
	if (cfgInfo->slotDelay < VTX_MIN_SLOTDELAY)
	{
		cfgInfo->slotDelay = VTX_MIN_SLOTDELAY;
	}

	//frame structure definition
	cfgInfo->fftSize = vtx_HALix_in->fftSize;
	cfgInfo->nbCp = ((int)VHW_DEFAULT_CPLENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->nbCp0 = ((int)VHW_DEFAULT_CP0LENGTH*cfgInfo->fftSize)/VHW_DEFAULT_FFT_SIZE;
	cfgInfo->fOffset_firstCarrier = vhw_calcFirstCarrier(cfgInfo->fftSize);
	cfgInfo->tOffset_preamble = (cfgInfo->fftSize+cfgInfo->nbCp)*VHW_PREAMBLE_SYMBOL + cfgInfo->nbCp0;
	cfgInfo->nbSubcarrier = vhw_calcNbSubcarrier(cfgInfo->fftSize);

	//preamble definition
	cfgInfo->preamble_length = vtx_HALix_in->preamble_length;
	memcpy(&(cfgInfo->preambleSeq[0]), vtx_HALix_in->preambleSeq, vtx_HALix_in->preamble_length * sizeof(int));

	//preamble insertion
	vtx_resetBufs();
	vtx_insertPreamble(cfgInfo);

	//DMRS insertion
	vtx_genDmrs(cfgInfo);


	LOG_E(PHY, "[vTX] configured and will start : mode %i, slotDelay : %i, preamble length : %i, fftSize : %i, CP: %i/%i, F_offset : %i, preamble time offset : %i, No. of subc : %i\n",
		cfgInfo->mode,
		cfgInfo->slotDelay,
		cfgInfo->preamble_length,
		cfgInfo->fftSize,
		cfgInfo->nbCp,
		cfgInfo->nbCp0,
		cfgInfo->fOffset_firstCarrier,
		cfgInfo->tOffset_preamble,
		cfgInfo->nbSubcarrier);
	
	return 0;
}



static void vtx_changeHwStats(vtx_rtCfg_t* rtCfgInfo, txHwStatus_e status)
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
static int vtx_allocateSymbol(vtx_cfgInfo_t* cfgInfo, int* dPtr, int remainLen, int slot_nr, int symb_nr)
{
	int prb_nr = 0;
	int k = cfgInfo->fOffset_firstCarrier;
	int allocatedLen = 0;
	int bitmap_dmrs[VHW_DMRS_NBSYMB];

	for (int i=0;i<VHW_DMRS_NBSYMB;i++)
		bitmap_dmrs[i] = 0;

	while (remainLen - allocatedLen > 0 && allocatedLen < cfgInfo->nbSubcarrier)
	{
		int cpyLen = remainLen - allocatedLen;
		int dmrs_nr = prb_nr/(HW_NB_SYM_IN_SLOT/2);

		if (cpyLen > 12)
			cpyLen = 12;

		//allocate a PRB
		memcpy(&vtx_freqBuffer[slot_nr][symb_nr][k], dPtr, cpyLen*sizeof(int));
		allocatedLen += cpyLen;

		//allocate DMRS
		if (bitmap_dmrs[dmrs_nr] ==  0)
		{
			vtx_insertDmrs(cfgInfo, slot_nr, prb_nr, dmrs_nr);
			bitmap_dmrs[dmrs_nr] = 1;
		}

		//data pointer shift
		dPtr = dPtr + cpyLen;

		//subcarrier offset shift
		k = (k + 12)%cfgInfo->fftSize;
		prb_nr++;
	}

	return allocatedLen;
}

//write to frequency buffer first
static void vtx_readBuffer(vtx_cfgInfo_t* cfgInfo, vtx_rtCfg_t* rtCfgInfo, uint32_t write_offset)
{
	uint32_t len, remainLen;
	int *dPtr;
	uint8_t symOffset;

	if (rtCfgInfo->slotSize-write_offset < rtCfgInfo->length[0])
	{
		len = rtCfgInfo->slotSize-write_offset;
	}
	else
	{
		len = rtCfgInfo->length[0];
	}

	if (len > 0)
	{
		remainLen = len;
		symOffset = 0;
		dPtr = rtCfgInfo->txData[0];
		while (remainLen > 0 && symOffset < HW_NB_SYM_IN_SLOT)
		{
			if ( (rtCfgInfo->slot_nr_sch == VHW_PREAMBLE_SLOT && symOffset == VHW_PREAMBLE_SYMBOL) || //preamble OFDM symbol : skip it
				(vhw_checkDmrsSymb(symOffset) == 1)) //DMRS symbol : skip it
			{
				symOffset++;
			}
			else
			{
				int allocatedLen = vtx_allocateSymbol(cfgInfo, dPtr, remainLen, rtCfgInfo->slot_nr_sch, symOffset);
				if (allocatedLen > 0)
					vtx_freqBufferBp[rtCfgInfo->slot_nr_sch][symOffset] = 1;
				remainLen -= allocatedLen;
				dPtr += allocatedLen;
				symOffset++;
			}
		}
	}
}



static void vtx_forceOff(void)
{
	AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_bufHal), "error on TX HAL MUTEX while forcing onOff");
	vtx_HALix_buf->onoff = 0;
	AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_bufHal), "error on TX HAL MUTEX while forcing onOff");
}



static uint8_t vtx_readOnOff(void)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_bufHal), "error on TX HAL MUTEX while reading onOff");
	onOff = vtx_HALix_buf->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_bufHal), "error on TX HAL MUTEX while reading onOff");

	return onOff;
}






void *vTX_mainThread(void *arg)
{
	static int __thread UE_thread_tx_retval;
	char threadname[128];
	
	uint8_t onOffCfg;
	vtx_cfgInfo_t cfgInfo;
	vtx_rtCfg_t rtCfgInfo;
	uint8_t sendFlag=0;
	hwIxStatus_e rfst;

	
	//register initialization
	vtx_HALix_in 	= &( ((ix_halTx_t*)arg)->woReg );
	vtx_HALix_out 	= &( ((ix_halTx_t*)arg)->roReg );
	vtx_HALix_buf 	= &( ((ix_halTx_t*)arg)->rwReg );

	vtx_muPtr_outHal = (pthread_mutex_t*)&(((ix_halTx_t*)arg)->roReg.mutex_roHal);
	vtx_muPtr_inHal = (pthread_mutex_t*)&(((ix_halTx_t*)arg)->woReg.mutex_woHal);
	vtx_muPtr_bufHal = (pthread_mutex_t*)&(((ix_halTx_t*)arg)->rwReg.mutex_rwHal);	
	
	vtx_csPtr_slotIrqHal = (pthread_cond_t*)&(((ix_halTx_t*)arg)->roReg.cond_roHal);
	vtx_csPtr_inHal = (pthread_cond_t*)&(((ix_halTx_t*)arg)->woReg.cond_woHal);
	vtx_csPtr_bufHal = (pthread_cond_t*)&(((ix_halTx_t*)arg)->rwReg.cond_rwHal);
	
	vtx_HALix_out->hwStatus = txst_null;

	//----------- Buffer initialization
	//main TX buffer for specific bank		
	for (int i=0;i<VHW_NB_SAMPLEBANK;i++)
		for (int j=0;j<VHW_NB_MAXSAMPLES;j++)
			vtx_buffer[i][j] = 0;

	//thread initialization
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	if ( threads.sync != -1 )
		CPU_SET(threads.sync, &cpuset);
	// this thread priority must be lower that the main acquisition thread
	sprintf(threadname, "Virtual TX");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY-1, &cpuset, threadname);

	//HW interface initialization
	// vs. RF
	AssertFatal ( NULL != (vtx_rfRegPtr = vrf_configRfReg()), "[vHW][ERROR] error in configuration RF-TX register : pointer is NULL!\n");
	
	
	printf("[vHW] TX virtual HW is initialized, waiting for on signal \n");
	do
	{
		usleep(100);
		AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->roReg.regMutex)), "");
		rfst = vtx_rfRegPtr->roReg.hwStatus;
		AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->roReg.regMutex)), "");
		
	} while (rfst == hwIx_null);

	printf("[vHW] TX virtual HW is starting now \n");


	vtx_changeHwStats(&rtCfgInfo, txst_off);
	vtx_HALix_out->slot_nr = 0;
	vtx_HALix_buf->sch_completed = 0;
	
	while (!oai_exit)
	{

		//off state loop (stay until on signal comes)
		AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_bufHal), "");
		while (vtx_HALix_buf->onoff == 0)
			pthread_cond_wait( vtx_csPtr_bufHal, vtx_muPtr_bufHal );
		AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_bufHal), "");
		
		//check on/off register and process it
		//if register is 'on', then do static configuration (init and start)
		vtx_configInit(&cfgInfo, &rtCfgInfo);
		vtx_changeHwStats(&rtCfgInfo, txst_idle);
		
		//inner loop for on operation
		while ( !oai_exit && 
				(rtCfgInfo.status != txst_off && 
				(onOffCfg = vtx_readOnOff()) == 1) )
		{
			switch (rtCfgInfo.status)
			{
				case txst_idle:

					//wait for the slot interrupt
					AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
					while ( rtCfgInfo.slot_nr_sch ==  (vtx_rfRegPtr->roReg.slot_nr + cfgInfo.slotDelay)%VHW_NB_SAMPLEBANK )
						pthread_cond_wait( &(vtx_rfRegPtr->sharedReg.irq_txrxInst), &(vtx_rfRegPtr->sharedReg.sharedMutex) ); // the thread waits here most of the time
					AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");


					AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
					rtCfgInfo.slot_nr_sch = (vtx_rfRegPtr->roReg.slot_nr + cfgInfo.slotDelay)%VHW_NB_SAMPLEBANK;
					rtCfgInfo.slotSize = vtx_rfRegPtr->roReg.slotSize;
					AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");

					LOG_D(PHY, "[vTX] slot IRQ from RF (%i)\n",rtCfgInfo.slot_nr_sch); 


					//indicate the slot number
					AssertFatal ( 0== pthread_mutex_lock(vtx_muPtr_outHal), "error on TX HAL MUTEX while writing out register");
					vtx_HALix_out->slot_nr = rtCfgInfo.slot_nr_sch;
					AssertFatal ( 0== pthread_mutex_unlock(vtx_muPtr_outHal), "error on TX HAL MUTEX while writing out register");

					//interrupt to upper layer
					AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_bufHal), "");
					vtx_HALix_buf->sch_completed = 0;
					pthread_cond_signal( vtx_csPtr_bufHal);
					AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_bufHal), "");


					//change status
					vtx_changeHwStats(&rtCfgInfo, txst_reading);
					break;
					
				case txst_reading:
					AssertFatal ( 0 == pthread_mutex_lock(vtx_muPtr_bufHal), "");
					while (vtx_HALix_buf->sch_completed != 1)
						pthread_cond_wait( vtx_csPtr_bufHal, vtx_muPtr_bufHal );
					vtx_HALix_buf->sch_completed = 2;
					AssertFatal ( 0 == pthread_mutex_unlock(vtx_muPtr_bufHal), "");


					if (vtx_checkInputData(&rtCfgInfo) == 1)
					{
						vtx_readBuffer(&cfgInfo, &rtCfgInfo, 0);
					}
					
					vtx_changeHwStats(&rtCfgInfo, txst_idle);

					break;
				
				default:
					break;
				
			}		
		
			if (rtCfgInfo.status == txst_idle)
			{
				sendFlag = 0;
				uint16_t symBitmap = 0;
				for (int i=0;i<HW_NB_SYM_IN_SLOT;i++)
				{
					if (rtCfgInfo.slot_nr_sch == VHW_PREAMBLE_SLOT && i == VHW_PREAMBLE_SYMBOL) //preamble symbol : skip it because it is already generated
					{
						sendFlag = 1;
						continue;
					}
					
					//decide which symbol to modulate
					if (vtx_freqBufferBp[rtCfgInfo.slot_nr_sch][i] == 1)
					{
						symBitmap = symBitmap | (0x01 << i);
						sendFlag = 1;
					}
				}

				if (sendFlag == 1)
				{
					//IFFT
					vtx_modSlot(&cfgInfo, rtCfgInfo.slot_nr_sch, symBitmap);

					//Send to RF
					AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
					memcpy(&vtx_rfRegPtr->sharedReg.txData[rtCfgInfo.slot_nr_sch][0][0], &vtx_buffer[rtCfgInfo.slot_nr_sch][0], rtCfgInfo.slotSize*sizeof(int));
					AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");

					vtx_clearBuf(&cfgInfo, (0x01 << rtCfgInfo.slot_nr_sch ));
					vtx_clearFreqBuf(&cfgInfo, (0x01 << rtCfgInfo.slot_nr_sch));
					sendFlag = 0;
				}
				else
				{
					AssertFatal ( 0== pthread_mutex_lock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
					memset(&vtx_rfRegPtr->sharedReg.txData[rtCfgInfo.slot_nr_sch][0][0], 0, rtCfgInfo.slotSize*sizeof(int));
					AssertFatal ( 0== pthread_mutex_unlock(&(vtx_rfRegPtr->sharedReg.sharedMutex)), "");
				}

			}

		}


		vtx_changeHwStats(&rtCfgInfo, txst_off);
	}  // while !oai_exit

	return &UE_thread_tx_retval;
}

