# 01_통신 신호 기본 이론 및 SDR개요
## 이론 및 실습 내용
### ***통신 신호 및 변조에 대한 기본 개념 & 기저대역 표현법 & SDR의 기본 개념 및 시스템 이해***


### **C언어를 이용해서 RF 신호처리에 대해 실습**

**: 통신 신호 및 변조에 대한 기본 개념 & 기저대역 표현법**

**: SDR의 기본 개념 및 시스템 이해(+SIMD병렬연산)**
            

**: USRP 개발 환경 구축(현재상황 Check)**

**: Code 보고 스스로 공부하기**


## 이론 
**1. 통신 신호 및 변조에 대한 기본 개념 & 기저대역 표현법**
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/01.jpg)

**2. SDR의 기본 개념 및 시스템 이해**
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/02.jpg)
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/03.jpg)
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/04.jpg)

**3. USRP 개발 환경 구축(현재상황 Check)**
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/05.jpg)



## 실습 (C언어)

**1. 코드 분석**
**: cdot_prod.c 코드 분석**

=> intel에서 제공하는 SIMD 병렬연산 함수를 코드를 분석해보자.
```
%  -- SIMD 병렬연산으로 64bit를 dot product해주는 함수

if defined(__x86_64__) || defined(__i386__)
  __m128i *x128,*y128,mmtmp1,mmtmp2,mmtmp3,mmcumul,mmcumul_re,mmcumul_im;
  __m64 mmtmp7;
  __m128i minus_i = _mm_set_epi16(-1,1,-1,1,-1,1,-1,1);
  int32_t result;

  x128 = (__m128i*) x;
  y128 = (__m128i*) y;

  mmcumul_re = _mm_setzero_si128();
  mmcumul_im = _mm_setzero_si128();

  for (n=0; n<(N>>2); n++) {
  
    // this computes Re(z) = Re(x)*Re(y) + Im(x)*Im(y)
    mmtmp1 = _mm_madd_epi16(x128[0],y128[0]);
    // mmtmp1 contains real part of 4 consecutive outputs (32-bit)

    // shift and accumulate results
    mmtmp1 = _mm_srai_epi32(mmtmp1,output_shift);
    mmcumul_re = _mm_add_epi32(mmcumul_re,mmtmp1);


    // this computes Im(z) = Re(x)*Im(y) - Re(y)*Im(x)
    mmtmp2 = _mm_shufflelo_epi16(y128[0],_MM_SHUFFLE(2,3,0,1));
    mmtmp2 = _mm_shufflehi_epi16(mmtmp2,_MM_SHUFFLE(2,3,0,1));
    mmtmp2 = _mm_sign_epi16(mmtmp2,minus_i);
    mmtmp3 = _mm_madd_epi16(x128[0],mmtmp2);
    // mmtmp3 contains imag part of 4 consecutive outputs (32-bit)

    // shift and accumulate results
    mmtmp3 = _mm_srai_epi32(mmtmp3,output_shift);
    mmcumul_im = _mm_add_epi32(mmcumul_im,mmtmp3);

    x128++;
    y128++;
  }
    // this gives Re Re Im Im
  mmcumul = _mm_hadd_epi32(mmcumul_re,mmcumul_im);

  // this gives Re Im Re Im
  mmcumul = _mm_hadd_epi32(mmcumul,mmcumul);

  // extract the lower half
  mmtmp7 = _mm_movepi64_pi64(mmcumul);
  // pack the result
  mmtmp7 = _mm_packs_pi32(mmtmp7,mmtmp7);
  // convert back to integer
  result = _mm_cvtsi64_si32(mmtmp7);
  
  _mm_empty();
  _m_empty();
 
  return(result);
 
```
(함수 이름)

mmcumul_re  - REAL TIME(실수부)구하는 함수

mmcumul_im - Imagine TIME(허수부)구하는 함수


(분석 내용)
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/06.jpg)


**(실수부+허수부) 최종 결과값**
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/07.jpg)


**2. 공부하면서 궁금했던 점(+답변)**

![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/08.jpg)
![](https://github.com/prizesilvers2/NR_Modulation/blob/main/Figs/09.jpg)

**3. 느낀점**

: 최적화 방법론이란 가장 많이 호출되는 부분을 가장 빠르게 처리해주는 것을 의미함.
SIMD병렬연산을 통해 CPU계산의 복잡도를 줄여주어서 프로그램을 최적화시켜줄 수 있었음.
이러한 과정을 통해, 프로그램을 할 때, 적합한 병렬화 대상을 찾아서 복잡도를 줄여줌으로써 성능개선을 할 수 있다는 사실을 깨달음. 
또한 이 덕분에 속도가 생명인 무선통신을 할 때 유용하게 사용된다는 것을 알고 앞으로하는 프로젝트에도 적용해볼 계획임.



