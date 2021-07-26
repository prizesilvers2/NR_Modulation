# Interim Conclusion
## Channel reciprocity analysis
### **Channel Estimation Method 1, 2, 3 & Results**

**: TDD 통신에서 Channel reciprocity를 확인하기위해 시도해본 3가지 방법 및 결과**


## Analysis Result ( MATLAB )

***1. Channel Estimation Method 1 & Results***

- Method1
> 채널 추정 방법1은 다음과 같다. 먼저 동기를 맞춘 후, RX는 Preamble detection을 성공한  RX ANT0의 Preamble DUMP DATA를 받고 멈춘다. 방법1과 같이 멈추게 되면, RX에서 preamble을 보내는 fdbk채널이 꺼지게 되므로 TX단은 Preamble detection을 실패하게 된다.
따라서 TX Preamble detection의 실패 2slot전으로 돌아가면 Preamble detection에 성공한 slot정보를 받을 수 있으므로 이때의 TX,  RX의 preamble dump를 받아 Channel estimation을 진행하였다.
>
> 아래 그림은 위의 설명을 정리한 그림이다.

<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_method1.png?raw=true" width="80%"></p>

- Result
> 평균 : 0.2767 		표준편차 : 2.3425		표본 : 400개
> -0.6044 2.5788
> QPSK Symbol을 사용하여 통신을 하였으므로 가로축은 tx와 rx의 phase의 차(f-domain)를 나타내며, 세로축은 확률을 의미한다.
>
> 이 그래프를 해석해보면, UL, DL 채널의 각도 차이는 평균 0에 가까운 가우스 분포를 띄는 것을 확인할 수 있고, 높은 확률로 UL,DL의 각도가 일치함을 확인할 수 있다. 또한 이는 인접한 시간 내 UL에서 추정한 CSI를 사용한다면, TX에서 인코딩 과정을 수행하는 STLC에서의 성능도 괜찮을 것이라고 추측하였다.
>
> **분포가 완벽하게 일치하지 않는다는 문제점이 발생함**
> 
> USRP의 특성상 System을 Assert를 시켰다고 바로 멈추지 않고 Assert 이후, 몇개의 slot을 더 보낸 후 멈출 수도 있다는 것을 발견하였다. 따라서 문제점을 데이터를 추출하는 방식이라고 생각하여 다른 방식으로 연속된 채널을 추정해보기로 하였다.

<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_result1.png?raw=true" width="80%"></p>


***2.  Channel Estimation Method 2 & Results***

- Method2
> 채널 추정 방법2은 다음과 같다. 먼저 동기를 맞춘 후, TX에서 DataNum이 1021일 때, 받은 preamble을 버퍼에 복사한다. 이후 RX에서 Decoding Data가 1021인 경우 RX는 멈추게 되고, TX에서는 DataNum이 1022가 되면, 이전에 복사한 preamble data의 dump를 받고 멈춘다. 이에 대한 더 자세한 매커니즘은 vThreadtx, vThreadrx 코드에서 설명해두었다.
> 
> 아래 그림은 위의 설명을 정리한 그림이다.
<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_method2.png?raw=true" width="80%"></p>

- Result
>
>
>
<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_result2.png?raw=true" width="80%"></p>


***3.  Channel Estimation Method 3 & Results***

- Method
> 
>
>
>
<p align="center"><img src="" width="80%"></p>

- Result
>
>
>

<p align="center"><img src="" width="80%"></p>
