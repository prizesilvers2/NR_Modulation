# NPHY_nr-ue
## vThread_L1 Protocol 설명
### ***Symbol or Bit Data Send Method***

### **SER or BER Performance calculation**

**: Bit Data를 보내기 위해 필요한 함수 및 변수 설명**

**: NPHY_thread_L1에서 Bit Data를 처리하는 과정**


## 코드 구현 ( C )

**1. Bit Data를 보내기 위해 필요한 함수 및 변수 설명**

**1-1) 변수 설명**

``` 
short txbitbuf[NPHY_FFTSIZE][NPHY_BITDATASize] ={0,};
short txbitbuf10[NPHY_BITDATASize]={0,};
```

**1-2) void gen_bitdata()**

=> Data를 10 bit로 0~1023을 표현하여 txbitbuf에 넣어줌

=> ex) 1023 = 1 1 1 1 1 1 1 1 1 1 로 표현


```
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
```

**2. NPHY_thread_L1에서 Bit Data를 처리하는 과정**

![]()


> Written with [StackEdit](https://stackedit.io/).

