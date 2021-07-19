/*
 * Bit Processing System Lab (BPS Lab, SMWU) Software
 *      
 */

//main 
#include "nr-uesoftmodem.h"

//platform
#include "NPAL_thread.h"

//stacks
#include "NPHY_nr-ue.h"
#include "NHAL_rf.h"
#include "NHAL_tx.h"
#include "NHAL_rx.h"
#include "NHAL_trxTypes.h"
#include "NPAL_engines.h"
#include "NPAL_time.h"

#define NPHY_NSUBC_SSB					240
#define NPHY_FIFO_PRIORITY   			40
#define NPHY_SERPRINT_RESOLUTION		1000

#define NPHY_FFTSIZE		1024
#define NPHY_BITDATASize	10

#define KHz (1000UL)
#define MHz (1000*KHz)


#define bitsend
//#define symbolsend
//#define DBGBER


typedef enum {
	mu_0 = 0,
	mu_1 = 1,
	mu_2 = 2,
	mu_3 = 3,
	mu_4 = 4,
	mu_max = 5
} NPHY_numerology_e;

static pthread_t nphy_pthreadL1;
openair0_config_t nphy_rfParams;

extern int 				ndType;
extern uint8_t 			bwMode;
extern uint16_t			preambleScale;
extern uint16_t			dataScale;
extern uint16_t			pilotScale;
extern uint8_t			dataCapture;
extern uint8_t				txMode;

extern uint32_t 		center_frequency;
int32_t                 uplink_frequency_offset[4];

typedef enum {TDD=1,FDD=0} duplex_t;

typedef struct eutra_band_s {
  int16_t band;
  uint32_t ul_min;
  uint32_t ul_max;
  uint32_t dl_min;
  uint32_t dl_max;
  duplex_t frame_type;
} eutra_band_t;

typedef struct band_info_s {
  int nbands;
  eutra_band_t band_info[100];
} band_info_t;



static const eutra_band_t eutra_bands[] = {
  { 1, 1920    * MHz, 1980    * MHz, 2110    * MHz, 2170    * MHz, FDD},
  { 2, 1850    * MHz, 1910    * MHz, 1930    * MHz, 1990    * MHz, FDD},
  { 3, 1710    * MHz, 1785    * MHz, 1805    * MHz, 1880    * MHz, FDD},
  { 4, 1710    * MHz, 1755    * MHz, 2110    * MHz, 2155    * MHz, FDD},
  { 5,  824    * MHz,  849    * MHz,  869    * MHz,  894    * MHz, FDD},
  { 6,  830    * MHz,  840    * MHz,  875    * MHz,  885    * MHz, FDD},
  { 7, 2500    * MHz, 2570    * MHz, 2620    * MHz, 2690    * MHz, FDD},
  { 8,  880    * MHz,  915    * MHz,  925    * MHz,  960    * MHz, FDD},
  { 9, 1749900 * KHz, 1784900 * KHz, 1844900 * KHz, 1879900 * KHz, FDD},
  {10, 1710    * MHz, 1770    * MHz, 2110    * MHz, 2170    * MHz, FDD},
  {11, 1427900 * KHz, 1452900 * KHz, 1475900 * KHz, 1500900 * KHz, FDD},
  {12,  698    * MHz,  716    * MHz,  728    * MHz,  746    * MHz, FDD},
  {13,  777    * MHz,  787    * MHz,  746    * MHz,  756    * MHz, FDD},
  {14,  788    * MHz,  798    * MHz,  758    * MHz,  768    * MHz, FDD},
  {17,  704    * MHz,  716    * MHz,  734    * MHz,  746    * MHz, FDD},
  {20,  832    * MHz,  862    * MHz,  791    * MHz,  821    * MHz, FDD},
  {22, 3510    * MHz, 3590    * MHz, 3410    * MHz, 3490    * MHz, FDD},
  {33, 1900    * MHz, 1920    * MHz, 1900    * MHz, 1920    * MHz, TDD},
  {34, 2010    * MHz, 2025    * MHz, 2010    * MHz, 2025    * MHz, TDD},
  {35, 1850    * MHz, 1910    * MHz, 1850    * MHz, 1910    * MHz, TDD},
  {36, 1930    * MHz, 1990    * MHz, 1930    * MHz, 1990    * MHz, TDD},
  {37, 1910    * MHz, 1930    * MHz, 1910    * MHz, 1930    * MHz, TDD},
  {38, 2570    * MHz, 2620    * MHz, 2570    * MHz, 2630    * MHz, TDD},
  {39, 1880    * MHz, 1920    * MHz, 1880    * MHz, 1920    * MHz, TDD},
  {40, 2300    * MHz, 2400    * MHz, 2300    * MHz, 2400    * MHz, TDD},
  {41, 2496    * MHz, 2690    * MHz, 2496    * MHz, 2690    * MHz, TDD},
  {42, 3400    * MHz, 3600    * MHz, 3400    * MHz, 3600    * MHz, TDD},
  {43, 3600    * MHz, 3800    * MHz, 3600    * MHz, 3800    * MHz, TDD},
  {44, 703    * MHz, 803    * MHz, 703    * MHz, 803    * MHz, TDD},
  {45, 3300    * MHz, 3500    * MHz, 3300    * MHz, 3500    * MHz, TDD},
  {99, 2400    * MHz, 2495    * MHz, 2400    * MHz, 2495    * MHz, TDD}
};


//M-Sequence (5G) -> use it for accurate frequency offset estimation
#define PREAMBLE_LENGTH		    128
short preamble_seq[PREAMBLE_LENGTH*2] = {4095, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 
										-4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 
										-4096, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 
										-4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 
										-4096, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, 
										4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 
										0, -4096, 0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 
										-4096, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 
										0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 
										0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 
										-4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 
										0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 
										-4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 
										-4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 
										-4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, 
										-4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, -4096, 0, 
										-4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, -4096, 0, -4096, 
										0, 4095, 0, -4096, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 
										0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 0, 
										-4096, 0, -4096, 0, -4096, 0, -4096, 0, 4095, 0, -4096, 0, 4095, 0, -4096, 
										0, 4095, 0, -4096, 0, 4095, 0, 4095, 0, 4095, 0, 4095, 0, -4096, 0, 0, 0};

int preamble_length = PREAMBLE_LENGTH;

short txbitbuf[NPHY_FFTSIZE][NPHY_BITDATASize] ={0,};
short txbitbuf10[NPHY_BITDATASize]={0,};

#ifdef bitsend
void gen_bitdata(void){
	
	int slotCnt = 0;
	int p0 = 0;
	int p1 = 0;
	int p2 = 0;
	int p3 = 0;
	int p4 = 0;
	int p5 = 0;
	int p6 = 0;
	int p7 = 0;
	int p8 = 0;
	int p9 = 0;

	for(int i=0; i<NPHY_FFTSIZE; i++)
	{

		p0 = slotCnt/512 ;
		p1 = (slotCnt %512)/256;
		p2 = ((slotCnt %512)%256)/128;
		p3 = (((slotCnt %512)%256)%128)/64;
		p4 = ((((slotCnt %512)%256)%128)%64)/32;
		p5 = (((((slotCnt %512)%256)%128)%64)%32)/16;
		p6 = ((((((slotCnt %512)%256)%128)%64)%32)%16)/8;
		p7 = (((((((slotCnt %512)%256)%128)%64)%32)%16)%8)/4;
		p8 = ((((((((slotCnt %512)%256)%128)%64)%32)%16)%8)%4)/2;
		p9 = ((((((((slotCnt %512)%256)%128)%64)%32)%16)%8)%4)%2;
			
		txbitbuf[i][0] = p0;
		txbitbuf[i][1] = p1;
		txbitbuf[i][2] = p2;
		txbitbuf[i][3] = p3;
		txbitbuf[i][4] = p4;
		txbitbuf[i][5] = p5;
		txbitbuf[i][6] = p6;
		txbitbuf[i][7] = p7;
		txbitbuf[i][8] = p8;
		txbitbuf[i][9] = p9;

		slotCnt = slotCnt +1;

		}
}
#endif

#ifdef symbolsend
short txdatabuf[100] = {23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169,
					   23169, -23169, 23169, 23169, -23169, -23169, -23169, 23169, 23169, 23169 };
#endif

int dataLength = 10;

static short hd_mapSym[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};


int NPHY_hardDecision(short* inputSym)
{
	int max_corr=-10000, corr_result;
	int max_index=-1;
	
	for (int k=0; k<4; k++)
	{
		corr_result = ((int)inputSym[0])*hd_mapSym[k][0] + ((int)inputSym[1])*hd_mapSym[k][1];
		
		if (max_corr < corr_result)
		{
			max_corr = corr_result;
			max_index = k;
		}
	}

	return max_index;
}


static void NPHY_calcFrameParam(int bwMode, double *samplingRate, double *bw, nhal_trxFFT_e *fftSize)
{
	switch (bwMode)
	{
		case 0: //5MHz
			*samplingRate = 7680000.0;
			*bw = 5.0e6;
			*fftSize = nhal_trxFFT_512;
			break;
			
		case 1 : //10MHz
			*samplingRate = 15360000.0;
			*bw = 10.0e6;
			*fftSize = nhal_trxFFT_1024;
			break;
			
		case 2 : //20MHz
			*samplingRate = 30720000.0;
			*bw = 20.0e6;
			*fftSize = nhal_trxFFT_2048;
			break;

		default:
			LOG_E(PHY, "Failed to set frame parameter : wrong bwmode (%i)\n", bwMode);
			
	}

}

static void NPHY_init_lowPHY(void);

uint8_t NPHY_decideNbAnt(void)
{
	uint8_t output = 1;

#if (defined MOD_STBC || defined MOD_STLC)
	if (ndType == 0) //RX
	{
		switch(txMode)
		{
			case 2: //STLC RX
				output = 2;
				break;
				
			case 1: //STBC RX
			case 0: //single
			default : 
				output=1;
		}
	}
	else
	{
		switch(txMode)
		{
			case 1: //STBC TX
				output = 2;
				break;
				
			case 2: //STLC TX
			case 0: //single
			default : 
				output=1;
		}
	}
#endif

	return output;
}



/// Subcarrier spacings in Hz indexed by numerology index
uint16_t nphy_slots_per_subframe[mu_max] = {1, 2, 4, 16, 32};
uint32_t nphy_subcarrier_spacing[mu_max] = {15e3, 30e3, 60e3, 120e3, 240e3};

static void *NPHY_thread_L1(void *arg) {
	static int __thread UE_thread_synch_retval;
	int i;
	char threadname[128];
	uint8_t slot_nr;
	nhal_trxFFT_e fftSizeConfig=0;
	int dataCnt=0;
	int cumErrCnt = 0;
	int nbAnt=1;
	int errCnt = 0;
	int ind;
	int lastPrint = 0;
	double rf_sampling_rate=0.0; 		// = 7680000
	double rf_tx_bw=0.0;						   // 10e6(orginal)
	nhal_rfMode_e rfMode;
	nhal_trxMode_e txrxMode;
	nhal_trxAnt_type_e antType = nhal_trxAnt_separated;

#ifdef bitsend	
	int DataNum = 0;					// 0 ~ 1023
	int detectNum = 0;					// finding dataNum

	gen_bitdata();					
#endif
	
#ifdef MOD_STLC
	uint16_t fdbk_pattern = 0; //bitmap of slots for feedback channel
	nhal_fdbk_width_e fdbk_width = nhal_fdbkWidth_1_2_1;
#endif

	NPHY_calcFrameParam(bwMode, &rf_sampling_rate, &rf_tx_bw, &fftSizeConfig);

	uint32_t samplePerSlot = (int)rf_sampling_rate/1000;	
	double rf_rx_bw = rf_tx_bw;

	// this thread priority must be lower that the main acquisition thread
	sprintf(threadname, "NRUEPHY");
	NPAL_init_thread(100000, 500000, NPHY_FIFO_PRIORITY-1, threadname);

	NHAL_RXcmd_init();
	NHAL_TXcmd_init();

	for ( ind=0; ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]); ind++)
	{		
		break;
		if ( eutra_bands[ind].dl_min <= center_frequency && eutra_bands[ind].dl_max >= center_frequency )
		{
			for (i=0; i<4; i++)
		  		uplink_frequency_offset[i] = eutra_bands[ind].ul_min - eutra_bands[ind].dl_min;

			break;
		}
	}
	AssertFatal( ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]), "Can't find EUTRA band for frequency");

	//scale down the data for low PAPR
	for (int i=0;i<preamble_length*2;i++)
	{
		if (preambleScale < 0)
			preamble_seq[i] = (preamble_seq[i]<<(-preambleScale));	
		else
		    preamble_seq[i] = (preamble_seq[i]>>preambleScale);
	}

	
#ifdef symbolsend	
	for (int i=0;i<dataLength*2;i++)
	{
		if (dataScale < 0)
		    txdatabuf[i] = (txdatabuf[i]<<(-dataScale));
		else
			txdatabuf[i] = (txdatabuf[i]>>dataScale);
	}
#endif

#ifdef bitsend
	//When bit send, scaling
	for(int j=0 ; j<NPHY_FFTSIZE; j++)
	{	
		for(int i=0; i<NPHY_BITDATASize; i++)
		{			
			if(txbitbuf[j][i]==1)
			{
				txbitbuf[j][i] = 5792;
			}
			else
			{
				txbitbuf[j][i] = -5792;
			}
		}
	}
#endif


//RF parameter configurations
	if (ndType == 0) //RX
	{
		rfMode = nhal_rfMode_rx;
	}
	else //TX
	{
		rfMode = nhal_rfMode_tx;
	}
//antenna configuration
	nbAnt = NPHY_decideNbAnt();
	
	txrxMode = nhal_trxMode_single;
//TRX configurations
#ifdef MOD_STBC
	if (txMode == 1)
	{
		txrxMode = nhal_trxMode_2x1STBC;
	}
#endif

#ifdef MOD_STLC
	if (txMode == 2)
	{
		txrxMode = nhal_trxMode_1x2STLC;
		antType = nhal_trxAnt_common;
		fdbk_pattern = 0xA; //bitmap of slots for feedback channel
		fdbk_width = nhal_fdbkWidth_3_6_5;
	}
#endif

	
	NHAL_RFcmd_on(NHAL_convSampleRate(rf_sampling_rate),
					samplePerSlot,
					center_frequency, center_frequency+uplink_frequency_offset[0],
					rf_rx_bw, rf_tx_bw, nphy_rfParams.rx_total_gain_dB, nphy_rfParams.rx_gain_offset[0], nphy_rfParams.tx_gain[0],
					rfMode, 
					nbAnt,
					antType
#ifdef MOD_STLC						
						,
					fdbk_pattern, //feedback slot
					fdbk_width //feedback channel configuration
#endif
					);

	if (ndType == 0) //vRX on
	{
		NHAL_RXcmd_on(txrxMode, preamble_length, (int*)preamble_seq, fftSizeConfig, dataLength);
	}
	else //vTX on
	{
		NHAL_TXcmd_on(txrxMode, preamble_length, (int*)preamble_seq, fftSizeConfig);
	}
	

  	while (!NPAL_checkEnd()) 
	{
		switch (ndType)
		{
			case 0: //RX node
				slot_nr = NHAL_RXcmd_waitForIrq();
				{
#ifdef symbolsend
					int16_t* dataPtr = (int16_t*)NHAL_RXcmd_getData(slot_nr, 2*dataLength);
					float snr;
					dataCnt += dataLength;
					
					//result calculation
					errCnt = 0;
					for (int i=0;i<dataLength/2;i++)
					{
						if (NPHY_hardDecision(&txdatabuf[2*i]) != NPHY_hardDecision(&dataPtr[2*i]))
						{
							errCnt++;
						}
					}
					
					cumErrCnt += errCnt;
					snr = NHAL_RXcmd_getSnr(0);

					if (dataCnt - lastPrint > NPHY_SERPRINT_RESOLUTION)
					{
						LOG_I(L1, "SER : %f (error/data : %i / %i), snr : %f\n", (float)cumErrCnt/dataCnt, cumErrCnt, dataCnt, snr);
						lastPrint = dataCnt;
					}
#endif

#ifdef bitsend
					int16_t* dataPtr = (int16_t*)NHAL_RXcmd_getData(slot_nr, dataLength);
					float snr;
					
					short dataCntbuf[NPHY_BITDATASize] ={0};
					
					dataCnt += dataLength;

					for(int i=0; i<5; i++)
					{
						int QPSKnum = NPHY_hardDecision(&dataPtr[2*i]);

						if(hd_mapSym[QPSKnum][0] == 1)
						{
							dataCntbuf[2*i] = 1;
						}
						else
						{
							dataCntbuf[2*i] = 0;
						}
						if(hd_mapSym[QPSKnum][1] == 1)
						{
							dataCntbuf[2*i+1] = 1;
						}
						else
						{
							dataCntbuf[2*i+1] = 0;
						}						
					}
					
					int decimal = 1;
					int total_decimal = 0;
					
					for(int position = 0; position<dataLength; position++)
					{
						if(dataCntbuf[position] == 1)
						{
							if(position == dataLength-1)
							{
								total_decimal = total_decimal + decimal;
							}
							else
							{
								 for(int count = 0; count < dataLength -(position+1); count++)
								 {
									decimal = decimal*2;
								 }

								 total_decimal = total_decimal + decimal;
								 decimal = 1;
							}
						}
					}


					//result calculation
					errCnt = 0;
					
					if(dataCnt == 10) 
						detectNum = total_decimal;
					
					for (int i=0; i<dataLength/2; i++)
					{
						int datanum = NPHY_hardDecision(&dataPtr[2*i]);
						
						if ((txbitbuf[detectNum][2*i]/5792) != hd_mapSym[datanum][0])
						{
							errCnt++;
						}

						if ((txbitbuf[detectNum][2*i+1]/5792) != hd_mapSym[datanum][1])
						{
							errCnt++;
						}
				
					}
					
					if(errCnt ==0)
					{
						detectNum = (detectNum+1)%NPHY_FFTSIZE;
					}
					else
					{
						#ifdef DBGBER
							printf(">>> errCnt :%d, cumErrCnt:%d, dataCnt:%d\n", errCnt, cumErrCnt, dataCnt);
						#endif
					
						detectNum = total_decimal;
						detectNum = (detectNum+1)%NPHY_FFTSIZE;
					}

					cumErrCnt += errCnt;
					snr = NHAL_RXcmd_getSnr(0);

					if (dataCnt - lastPrint > NPHY_SERPRINT_RESOLUTION)
					{
						LOG_I(L1, "BER : %f (error/data : %i / %i), snr : %f\n", (float)cumErrCnt/dataCnt, cumErrCnt, dataCnt, snr);
						lastPrint = dataCnt;
					}

					#ifdef DBGBER
						printf(">>> [NPHY] Slot_nr : %d, total_decimal : %d\n", slot_nr, total_decimal);
						printf("\n");
					#endif

#endif
				}
				
				NHAL_RXcmd_clrDecIrq();
				
				break;

			case 1: //TX node
				
				slot_nr = NHAL_TXcmd_waitForIrq();

#ifdef symbolsend
				NHAL_TXcmd_cfgTxData(dataLength, txdatabuf);
				//NHAL_TXcmd_cfgTxData(0,NULL); // this code should be used when not sending any data (or NULL data)
#endif

#ifdef bitsend	
				LOG_I(PHY, "slot_nr : %d , DataNum : %d \n", slot_nr, DataNum);
#ifdef MOD_STLC
				if((slot_nr%2)==0)
				{
					memcpy(&txbitbuf10[0], &txbitbuf[DataNum][0], NPHY_BITDATASize*sizeof(short));
					DataNum = (DataNum+1)%NPHY_FFTSIZE;
				}
#else
				memcpy(&txbitbuf10[0], &txbitbuf[DataNum][0], NPHY_BITDATASize*sizeof(short));
				DataNum = (DataNum+1)%NPHY_FFTSIZE;
#endif

				NHAL_TXcmd_cfgTxData(dataLength,txbitbuf10);
#endif

				NHAL_TXcmd_startTx();

				break; 

			default:
				break;

		}
	}  // while !oai_exit

	NHAL_TXcmd_off();
	NHAL_RXcmd_off();
	NHAL_RFcmd_off();

  	return &UE_thread_synch_retval;
}

void NPHY_initHW(void)
{
	uint8_t nbAnt = NPHY_decideNbAnt();
	
	AssertFatal( 0 == NHAL_RFcmd_init(nphy_rfParams.mmapped_dma, nphy_rfParams.clock_source, nphy_rfParams.configFilename, nphy_rfParams.sdr_addrs, nbAnt), 
						"RF HW cannot be initialized properly\n");
		
	AssertFatal( 0 == NHAL_RXcmd_init(),
					"RX HW cannot be initialized properly\n");
	
	AssertFatal( 0 == NHAL_TXcmd_init(),
					"TX HW cannot be initialized properly\n");	
}




void NPHY_init(void* arg)
{
	openair0_config_t* rf_params = (openair0_config_t*)arg;
	
	//L1 scheduler initialization
	pthread_create(&nphy_pthreadL1,NULL,NPHY_thread_L1,NULL);

	//algorithm initialization
	memcpy(&nphy_rfParams, rf_params, sizeof(openair0_config_t));
	NPHY_init_lowPHY();
}

static void NPHY_init_lowPHY(void)
{
	crcTableInit();
	init_dfts();

	//frame parameter configuration
	NPHY_initHW();
}

