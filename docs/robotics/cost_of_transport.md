---
layout: default
title: Cost of ransport
last_modified_dtate: 2023-11-12 20:45:24
parent: Robot Control
grand_parent: Robotics
nav_order: 1
permalink: /docs/robotics/robot_control/cost_of_transport/
---

Cost of Transport
{: .fs-7 .fw-700 }

Cost of Transport(CoT)는 로봇이 이동하는 데 필요한 에너지의 양을 나타낸다. 정확히는 로봇의 단위 질량(unit mass)을 단위 거리(unit distance)만큼 이동시키는 데 필요한 일(work) 또는 에너지 양으로, 로봇이 얼마나 에너지 효율적으로 걷는지 평가하는 지표이다. 특히 생물 모방을 기본으로 하는 bio robotics 분야에서, CoT를 통해 로봇 시스템과 생물학적 개체의 에너지 효율성을 직접 비교할 수 있다. 이러한 비교는 생물학적 이동 효율성을 맞추거나 능가할 수 있는 로봇을 만드는 데 중요하게 사용된다.   

Work를 $$W$$, stride length를 $$\lambda$$, 로봇의 질량을 $$m$$이라고 할 때 CoT $$T$$는 다음과 같이 구할 수 있다.  

$$ T = \frac{W}{m\lambda} $$  

이 식을 기본으로, ground에서 몇가지 로봇 locomotion의 CoT를 알아보자.

<br/>

{: .highlight-title }
> Note   
> 
> - **Stride:** A complete cycle of movement, e.g. from the setting down a foot to the next setting down of the same foot. 한 발 걷고 두 발 걸으면 init position으로 돌아옴. 한 cycle.
> - **Stride length:** Distance treveled in ont stride

<br/>

---

Crawling, two anchor
{: .fs-6 .fw-700 }

평평한 바닥에서 애벌레와 같이 two anchor crawling할 경우 발생하는 힘은 마찰력 $$ F = \mu_{forward} N $$이며 CoT $$T_{friction}$$ 는 다음과 같다.  

$$ T_{friction} = \frac{W}{m\lambda} = \frac{\mu_{forward} N \lambda}{m\lambda} = \frac{\mu_{forward}mg\lambda}{m\lambda} = \mu_{forward}g $$   

<br/>

또한 일은 에너지 변화량이고 crawling할 때 변화하는 에너지는 운동에너지이므로, 아래와 같은 과정을 통해 CoT를 운동에너지 변화량을 통해 표현할 수도 있다.   
1. **Simplification:**   
   think body as consisting of 3 equal parts
2. **For steady crawling at speed $$v$$:**  
   * Middle part: forward motion with speed $$v$$
   * Front and rear parts: stationary at half the time, move forward with the speed $$2v$$ at the other half   
3. **Work at $$\lambda$$**:   
   $$W = \frac{1}{2} \frac{2m}{3} (2v)^{2} = \frac{4mv^{2}}{3}$$  

Thus,   
   
$$ T_{inertia} = \frac{W}{m\lambda} = \frac{\frac{4mv^{2}}{3}}{m\lambda} = \frac{4v^{2}}{3\lambda} $$

<br/>

---

Crawling, serpentine
{: .fs-6 .fw-700 }

뱀처럼 움직이는 locomotion을 serpentine locomotion이라고 한다(All parts of the body move simultaneously, experiencing continuous sliding contact with the ground).

<br/>

---
Walking
{: .fs-6 .fw-700 }

일정한 속도로 걷는다면, 한 일은 0인데도 왜 사람은 걸을 때 힘이들까? 바로 무게중심이 위 아래로 이동하기 때문이다. Walking의 CoT도 이 무게중심의 변화를 고려해야한다.

(사진)
 
 위 그림은 walking을 가장 단순하게 modeling한 것이다. 무게중심은 
 
$$ T_{inertia} = \frac{W}{m\lambda} = \frac{\frac{1}{2} m (v\sin \theta)^2 (2)}{m\lambda} = \frac{v^{2}}{3\lambda} $$

