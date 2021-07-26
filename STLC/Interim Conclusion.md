# Interim Conclusion
## Channel reciprocity analysis
### **Channel Estimation Method 1, 2, 3 & Results**

**: TDD 통신에서 Channel reciprocity를 확인하기위해 시도해본 3가지 방법 및 결과**

**: Channel Non-reciprocity관련 논문**

## Analysis Result ( MATLAB )

***1. Channel Estimation Method 1 & Results***

- Method1
> 채널 추정 방법1은 다음과 같다. 먼저 동기를 맞춘 후, RX는 Preamble detection을 성공한  RX ANT0의 Preamble DUMP DATA를 받고 멈춘다. 방법1과 같이 멈추게 되면, RX에서 preamble을 보내는 fdbk채널이 꺼지게 되므로 TX단은 Preamble detection을 실패하게 된다.
따라서 TX Preamble detection의 실패 2slot전으로 돌아가면 Preamble detection에 성공한 slot정보를 받을 수 있으므로 이때의 TX,  RX의 preamble dump를 받아 Channel estimation을 진행하였다.
>
> 아래 그림은 위의 설명을 정리한 그림이다.

<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_method1.png?raw=true" width="80%"></p>

- Result
> 평균 : 0.2767	표준편차 : 2.3425	표본 : 400개
> 
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
> 평균 : -0.6044	표준편차 : 2.5788	표본 : 2000개
>  
> QPSK Symbol을 사용하여 통신을 하였으므로 가로축은 tx와 rx의 phase의 차(f-domain)를 나타내며, 세로축은 확률을 의미한다.
>
> 이 그래프를 통해 방법1과 마찬가지로 UL, DL 채널의 각도 차이는 평균 0에 가까운 가우스 분포를 띄는 것을 확인할 수 있고, 방법1보다 높은 확률로 UL, DL의 각도가 일치함을 확인할 수 있다. 

<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_result2.png?raw=true" width="80%"></p>

> **문제점 : Channel Non-reciprocity한 상황**
> 
> Channel Non-reciprocity의 원인을 2가지로 추정해보았다. 
>
> 첫번째 원인은 RX에서는 frequency offset을 보정해주는 코드가 있는 반면, TX에서는 frequency offset을 보정해주는 코드가 없어서 이러한 오차 값이 발생하였다고 생각하였다. 따라서 방법3에서는 TX의 frequency offset을 보정해줄 예정이다.
>
> 두 번째로 Channel Non-reciprocity의 원인은 TDD방식으로 통신을 할 때 Preamble Symbol끼리 1ms의 시간 차이가 나기때문에 그동안 phase가 많이 돌아가서 생기는 frequency offset이라고 생각하였다. 현재 sampling frequency가 15360Hz로 매우 큰 상황이기 때문에 frequency offset의 phase가 작아도 1ms마다 돌아가는 phase가 매우 클 것이라고 추측하였다. 대략적으로 시간차이가 1ms이 날 때, frequency offset이 미치는 영향을 살펴보면 일반적으로 안정적이라고하는 frequency offset을 -300 ~ 300Hz라고 할 때 각도의 차이는 -1.89~1.89 rad으로 크게 차이 나는 것을 확인할 수 있다.
>
> 아래 그림은 두 번째로 생각한 원인을 그려서 나타낸 그림이다.

<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_reason.png?raw=true" width="80%"></p>


***3.  Channel Estimation Method 3 & Results***

- Method
> Data를 추출하는 방식은 Channel Estimation Method 2와 동일하다. 방법3은 방법2와 다르게 TX와 RX 모두에서 Frequency offset을 모두 보정해준 뒤 실험을 진행하였다. 또한, 감쇄기(20dB)를 단 후, 유선 상황에서 채널을 추정해보았다.


- Result
> 평균 : -0.2993	표준편차 : 2.5428	표본 : 1000개
>
> QPSK Symbol을 사용하여 통신을 하였으므로 가로축은 tx와 rx의 phase의 차(f-domain)를 나타내며, 세로축은 확률을 의미한다.
>
> 이 그래프를 통해 방법1과 마찬가지로 UL, DL 채널의 각도 차이는 평균 0에 가까운 가우스 분포를 띄는 것을 확인할 수 있고, 방법1, 2 보다 높은 확률로 UL, DL의 각도가 일치함을 확인할 수 있다. 
>
> **그러나 여전히 Channel Non-reciprocity한 상황임**
> 
> Channel reciprocity를 구현하기 위한 추가적인 연구가 필요하다. 

<p align="center"><img src="https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/channelestimation_result3.png?raw=true" width="80%"></p>



***참고) Channel Non-reciprocity관련 논문***

- Senay Haile, "Investigation of Channel Reciprocity for OFDM TDD Systems",  Waterloo, Ontario, Canada, 2009

- Maxime Guillaud, Florian Kaltenberger, "Towards Practical Channel Reciprocity Exploitation: Relative Calibration in the Presence of Frequency Offset", IEEE Wireless Communications and Networking Conference(WCNC): PHY, 2013
