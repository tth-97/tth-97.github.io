---
layout: default
title: A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning
last_modified_date: 2023-11-20 15:12:24
parent: Papers
nav_order: 1
permalink: /docs/papers/dagger
---

A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning
{: .fs-7 .fw-700 }

[A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning](https://proceedings.mlr.press/v15/ross11a.html) (PMLR 2011)

<br/>

---
 
Dataset aggregation
{: .fs-6 .fw-700 }

DAGGER 알고리즘의 핵십은 학습 과정에서 지속적으로 데이터를 수집하고, 이를 기반으로 학습된 정책을 개선하는 것이다. 이 과정은 모델이 다양한 상황에 대응할 수 있도록 하며, 특히 expert의 지식을 모델에 효과적으로 전달하는 데 중점을 둔다. 초기에는 expert 정책의 영향을 많이 받지만, 점차 학습된 정책의 비중이 커지면서 독립적인 결정을 내리는 방향으로 진행된다.  

<br/>

![sigmoid_function](../../../../assets/images/papers/dagger.png){: width="70%" height="70%"}

<br/>

1. 초기화 단계:  
  * $$\mathcal{D} \ \gets \ \phi$$: 빈 데이터셋 $$\mathcal{D}$$를 초기화한다. 이 데이터셋은 학습 과정에서 수집된 데이터를 저장하는 데 사용된다.  
  * $$\hat{\pi_{1}}$$: 임의의 정책 $$\hat{\pi_{1}}$$을 초기화한다. 이 정책은 학습의 시작점이 된다.  
2. 반복 학습 과정(N회 반복): 
  * $$\pi_{i} = \beta_{i}\pi^{*} + (1-\beta_{i})\hat{\pi_{i}}$$: 혼합 정책 $$\pi_{1}$$를 생성한다. 여기서 $$\pi_{*}$$는 expert 정책(기존에 트레이닝된 정책)을 의미하고,  $$\hat{\pi_{i}$$는 현재 학습된 정책을 의미한다. $$\beta_{i}$$는 두 정책을 혼합하는 배율을 결정한다.   
  * Sample T-step trajectories using $$\pi_{i}$$: 혼합 정책 $$\pi_{i}$$를 사용하여 T스텝의 trajectories를 샘플링한다.  
  * Get dataset $$\mathcal{D_{i}} = \left\{ (s, \pi^{*}(s)) \right\}$$: $$\pi_{i}$$가 방문한 states의 집합과, 각 states에서 expert 정책이 제공하는 action을 기록하여 데이터셋 $$\mathcal{D_{i}}$$를 생성한다.  
  * $$\mathcal{D} \ \gets \ \mathcal{D} \cup \mathcal{D_{i}}$$: 수집된 데이터셋 $$\mathcal{D_{i}}$$를 기존 데이터셋 $$\mathcal{D}$$에 추가한다.  
  * Train classifier $$\hat{\pi_{i+1}}$$ on $$\mathcal{D}$$: 새로운 데이터셋 $$\mathcal{D}$$를 사용하여 classifier(정책) $$\hat{\pi_{i+1}}$$를 트레이닝한다.  
3. 결과 반환:
  * Return best $$\hat{\pi_{i}}$$ on validation: 검증 과정을 통해 가장 성능이 좋은 정책 $$\hat{\pi_{i}}$$를 선택하여 반환한다.

참고  
PyTorch 문서: [https://pytorch.org/docs/stable/generated/torch.cat.html](https://pytorch.org/docs/stable/generated/torch.cat.html)   
