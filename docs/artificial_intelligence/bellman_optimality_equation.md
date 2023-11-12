---
layout: default
title: Bellman Optimality Equation
last_modified_date: 2023-11-01 21:55:00
parent: Reinforcement Learning
grand_parent: Artificial Intelligence
nav_order: 3
permalink: /docs/artifirial_intelligence/reinforcement_learning/bellman_optimality_equation/
---

Bellman Optimality Equation
{: .fs-7 .fw-700 }

**Bellman optimality equation(MDP)**, 즉 벨만 최적 방정식은  **discrete time stochastic control process**이다. 이는 결과가 partly random && partly under the control of a decision maker한 상황에서 의사결정을 모델링하기 위한 수학적 틀이다.  MDP는 dynamic programming을 통해 해결되는 최적화 문제의 도구로 활용되며,  로봇 공학, 제어 자동화, 경제학, 제조업 등 많은 학문에서 사용된다. MDP는 markov chain의 확장으로, markov chain에서 actions(allowing choice)과 rewards(giving motivation)이 추가된 것이다. 반대로, 각 상태에 대해 오직 하나의 action만이 존재하고(e.g. "wait") 모든 rewards가 같다면(e.g. "zero"), MDP는 markov chain으로 간소화 될 수 있다. 

<br/>

{: .highlight-title }
> Note   
>   
> **Markov chain**   
>   
> A Markov chain or Markov process is a stochastic model describing a sequence of possible events in which the probability of each event depends only on the state attained in the previous event. Informally, this may be thought of as, "What happens next depends only on the state of affairs now."

<br/>

---

Definition
{: .fs-6 .fw-700 }
 
MDP는 다음 네 가지 주요 구성 요소로 이루어져 있다:

* States
�
S): 의사결정자가 인지할 수 있는 상황 혹은 위치.
행동 (Actions, 
�
A): 의사결정자가 선택할 수 있는 동작이나 전략.
전이 확률 (Transition Probabilities, 
�
P): 특정 상태에서 특정 행동을 취했을 때 다른 상태로 이동할 확률.
보상 함수 (reward function, 
�
r): 특정 상태에서 특정 행동을 취했을 때 받을 수 있는 보상의 기댓값.

매 time step에서, process는 어떤 상태 $$s$$에 있고, 의사결정자는 상태 $$s$$에서 가능한 어떤 행동 $$a$$를 선택할 수 있다. 다음 time step에서 process는 새로운 상태 $$s'$$로 무작위로 이동하며, 의사결정자에게 해당 보상 $$R_a(s, s;)$$를 제공한다.   
  
Process가 새로운 상태 $$s'$$로 이동하는 확률은 선택한 action에 의해 영향을 받는데, 이는 state transition function $$P_a(s, s')$$에 의해 주어지는 값이>다. 즉, 다음 상태 $$s'$$는 현재 상태 $$s$$와 의사결정자의 action $$a$$에 따라 달라진다. 그러나 $$s$$와 $$a$$는 모든 이전 상태와 행동과 conditionally independent하다. 다시 말해, MDP의 상태 전이는 마르코프 특성(현재 상태와 그 상태에서 취하는 행동만이 다음 상태와 보상에 영향을 미친다는 뜻이다. 과거의 >상태나 행동은 현재의 의사결정에 영향을 주지 않는다.)을 만족한다.   
  

<br/>
---

참고  
WIKIPEDIA: [https://en.wikipedia.org/wiki/Markov_decision_process](https://en.wikipedia.org/wiki/Markov_decision_process)   
