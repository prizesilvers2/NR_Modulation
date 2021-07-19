# 02_Preamble Design
## 이론 및 실습 내용
### ***디지털 통신에서의 preamble 역할에 대해 이해***

(이론)

**: preamble 개념 및 동기화 개념 이해**

**: LAB의 전체구조(4-Thread구성)**

(실습)

**: TX-RX간 preamble 송수신을 통해 동기화를 하는 부분 구현**

**TX - preamble전송 부분 구현** 

**RX- preamble detection을 구현하여 수신여부 확인**


## 이론 
**1. preamble 개념 및 동기화 개념 이해**

Synchronization(동기화)

Preamble()
LTE - Zhadoff Chu sequence
NR - M sequence

NR - Frame structure()
frame boundary
correlation -> cross-correlation - inner product이용
=> timing을 잡을 수 있음. 
(3GPP 표준)

(실제 LAB에서 사용하는 structure)

**2. LAB의 전체구조 및 역할(4-Thread구성)**
(전체 Structure)
![]()

(Time sequence diagram으로 data 처리 순서 알아봄)
Virtual RF
Virtual TX
Virtual RX
PHY



## 실습 (C언어)

: TX-RX간 preamble 송수신을 통해 동기화를 하는 부분 구현

**1. TX실습**
**: vThread_tx.c 코드 구현**

=> MATLAB으로 아래의 Preamble Seq를 구현한 뒤 nr-ue.c의 preamble에 넣어보고 성능 비교해보기.

(수식을 참고하여 MATLAB으로 구한 값)
LTE - Zhadoff Chu sequence
![]()
NR - M sequence
![]()

(넣은 코드)
```
%  -- preamble seq 구현
// m - sequence
#if 1
#define PREAMBLE_LENGTH		128
int preamble_length = PREAMBLE_LENGTH;
short preamble_seq[PREAMBLE_LENGTH*2]={32767,0,-32768,0,-32768,0,32767,0,-32768,0,-32768,0,-32768,0,-32768,0,32767,0,32767,0,-32768,0,-32768,0,-32768,0,32767,0,32767,0,
	-32768,0,32767,0,-32768,0,32767,0,-32768,0,-32768,0,32767,0,32767,0,-32768,0,-32768,0,32767,0,32767,0,32767,0,32767,0,32767,0,-32768,0,-32768,0,32767,0,-32768,0,
	-32768,0,32767,0,-32768,0,32767,0,-32768,0,-32768,0,-32768,0,32767,0,-32768,0,32767,0,32767,0,32767,0,-32768,0,-32768,0,32767,0,32767,0,-32768,0,32767,0,32767,0,
	32767,0,-32768,0,32767,0,32767,0,32767,0,32767,0,32767,0,32767,0,-32768,0,32767,0,32767,0,-32768,0,32767,0,32767,0,-32768,0,-32768,0,32767,0,-32768,0,32767,0,32767,0,
	-32768,0,-32768,0,-32768,0,-32768,0,32767,0,-32768,0,-32768,0,-32768,0,32767,0,32767,0,32767,0,32767,0,-32768,0,-32768,0,-32768,0,-32768,0,-32768,0,-32768,0,-32768,0,
	32767,0,32767,0,32767,0,-32768,0,-32768,0,-32768,0,32767,0,-32768,0,-32768,0,32767,0,32767,0,32767,0,-32768,0,32767,0,-32768,0,32767,0,32767,0,-32768,0,32767,0,-32768,
	0,-32768,0,-32768,0,-32768,0,-32768,0,32767,0,-32768,0,32767,0,-32768,0,32767,0,-32768,0,32767,0,32767,0,32767,0,32767,0,-32768,0};

// Zadoff-Chu Sequence
#else
#define PREAMBLE_LENGTH		72
int preamble_length = PREAMBLE_LENGTH;
short preamble_seq[PREAMBLE_LENGTH*2]={0,0,0,0,0,0,0,0,0,0,32767,0,-26120,-19785,11971,-30502,-24020,-22287,32117,6493,31311,9658,
	-16383,-28377,25101,-21062,-7291,-31945,20430,25618,14949,29158,11971,-30502,31311,9658,25101,-21062,-16383,28377,-24020,22287,
	32117,6493,-7291,31945,20430,25618,-26120,-19785,-16384,-28377,-16384,28377,-26120,-19785,-32401,4884,31311,-9658,32117,6493,-7291,
	-31945,32767,0,25101,-21062,-24020,22287,-32401,4884,-32401,4884,-24020,22287,25101,-21062,32767,0,-7291,-31945,32117,6493,31311,-9658,
	-32401,4884,-26120,-19785,-16383,28377,-16384,-28377,-26120,-19785,20430,25618,-7291,31945,32117,6493,-24020,22287,-16384,28377,25101,
	-21062,31311,9658,11971,-30502,14949,29158,20430,25618,-7291,-31945,25101,-21062,-16384,-28377,31311,9658,32117,6493,-24020,-22287,11971,
	-30502,-26120,-19785,32767,0,0,0,0,0,0,0,0,0,0,0};

#endif
 
```
(Zhadoff Chu sequence  VS  M sequence) 
=> MATLAB구현시 차이점 그래프로 비교해보기

Zhadoff Chu sequence
![]()
M sequence
![]()

(+ 어떤과정으로 nr-ue.c에서 tx로 넘어가고 preamble이 생성되며 전송되는지 코드로 확인하기)
=> 어떤 함수가 호출되는지 과정 살펴보기





**2. RX실습**
**: vThread_rx.c 코드 분석**

=> vrx_genPreamble 함수 구현
- preamble seq를 가운데 주파수에 놓고 IFFT
(여기서 preamble을 넣을 때 들어가는 순서가 특이한데 이러한 이유 설명하기)

(+ 어떤과정으로 rx에서 preamble을 찾는지 코드로 확인하기)
=> 어떤 함수가 호출되는지 과정 살펴보기



**3. 공부하면서 궁금했던 점(+답변)**


**4. 느낀점**

: 통신기법(LTE->NR)이 바뀜에 따라 사용하는 preamble seq가 다르다는 것을 알게 됨.

: multi thread는 처음

 