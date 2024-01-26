---
layout: default
title: Robot Parkour Learning
last_modified_date: 2023-11-15 15:12:24
parent: Papers
nav_order: 2
permalink: /docs/papers/robot_parkour_learning
---

Robot ParKour Learning
{: .fs-7 .fw-700 }

[Robot Pakour Learning](https://robot-parkour.github.io/resources/Robot_Parkour_Learning.pdf) (CoRL 2023)

<br/>

---
 
Robot Parkour Learning Systems
{: .fs-6 .fw-700 }

1. Parkour Skills Learning via Two-Stage RL
{: .fs-5 .fw-700 }

Robot vision 사용해야하는 모션을 학습할 때, 보통 depth/rgb iamge data를 direct하게 RL로 학습시킨다. 그러나 이 방법은 불안정하고, render하는데 costly하다는 단점이 있다. 따라서 이 논문에서는 **previleged visual information**을 사용한다.

* **Previleged visual information:** 
   - the distance from the robot's current position to the obstacle in front of the robot
   - the height of the obstacle
   - the width of the obstacle
   - a 4-dimensional one-hot category representing the four types of obstacles
  
  
RL 요소 정의


* **Inputs of policy(states):**
  * $$ S_{t}^{propprio} \in \mathbb{R}^{29} $$ (row, pitch, base angular velocities, joints positions, joints velocities)
  * last action $$a_{t-1} \in \mathbb{R}^{12} $$
  * the privileged visual information $$e_{t}^{vis}$$
  * privileged physics information $$e_{t}^{phy}$$

<br/>

* **Outputs of policy:**
  * target joint positions $$a_{t} \in \mathbb{R}^{12}$$

<br/>

* **Environment:**

각각의 specialized skill plicies  $$\pi_{climb}, \pi_{leap}, \pi_{crawl}, \pi_{tilt}, \pi_{run}$$ 는 아래와 같은 terrains에서 학습하였다.

![sigmoid_function](../../../../assets/images/papers/dynamics_constraint.png){: width="100%" height="100%"}

<br/>

* **reward:**

모든 specialized skill poicies  $$\pi_{climb}, \pi_{leap}, \pi_{crawl}, \pi_{tilt}, \pi_{run}$$ 는 같은 reward 구조를 사용하여 학습하였다. (reward engineering이 없는 것이 메리트인 듯) reward는 mechanical energy를 최소화하면서 general한 skill을 자연스러운 모션으로 생성해 낼 수 있도록 구성되었고, 그 식은 아래와 같다.  
  
$$ r_{skill} = r_{forward} + r_{energy} + r_{alive},$$    
   
where  
    
$$ r_{forward} = -\alpha_{1} * |v_{x} - v_{x}^{target}| - \alpha_{2} * {|v_{y}|}^2 + \alpha_{3} * e^{-|w_{yaw}|},$$   
$$ r_{energy} = -\alpha_{4} * \sum_{j \in joints} {|\tau_{j}\dot{q}_{j}|}^2, $$   
$$ r_{alive} = r $$  

위 리워드는 매 step마다 계산되며, $$v_{x}$$는 forward base linear velocity, $$v_{x}^{target}$$은 target speed(대략 1 m/s), $$v_{y}$$는 lateral base linear velocity, $$\omega_{yaw}$$는 base angular yaw velocity, $$\tau_{j}$$는 torque at joint $$j$$, $$\dot{q}_{j}$$는 joint velocity at at joint j 그리고 $$\alpha$$는 are hyperparameters이다.



{: .fs-5 .fw-700 .text-yellow-200 }

dim=0 은 첫 번째 차원을 기준으로 텐서들을 합치는 것을 의미한다. 두 개의 2x3 텐서를 첫 번째 차원을 기준으로 합치면 (2+2)x3 = 4x3 텐서가 된다.

```python
x = torch.tensor([[1,  2,  3], 
                  [4,  5,  6]])
y = torch.tensor([[7,  8,  9],
                  [10, 11, 12]])
result = torch.cat((x, y), dim=0) # tensor([[1,  2,  3], 
			          #	    [4,  5,  6],
				  #	    [7,  8,  9],
				  #	    [10, 11, 12]])
```

<br/>
* dim=1
{: .fs-5 .fw-700 .text-yellow-200 }

dim=1 은 두 번재 차원을 기준으로 텐서들을 합치는 것을 의미한다. 두 개의 2x3 텐서들을 두 번째 차원을 기준으로 합치면 2x(3+3) = 2x6 텐서가 된다.

```python
x = torch.tensor([[1,  2,  3], 
                  [4,  5,  6]])
y = torch.tensor([[7,  8,  9],
                  [10, 11, 12]])
result = torch.cat((x, y), dim=1) # tensor([[1,  2,  3,  7,  8,  9],
                                  #         [4,  5,  6, 10, 11, 12]])
```

<br/>
* 특정 행에 값 삽입하기
{: .fs-5 .fw-700 .text-yellow-200 }

```python
# 초기 텐서 생성
x = torch.tensor([[1, 2], [3, 4], [5, 6]])

# 삽입할 텐서 생성
new_row = torch.tensor([[7, 8]])

# 특정 위치에 행 삽입 (예: 두 번째 행 다음)
index_to_insert = 2
result = torch.cat((tensor[:index_to_insert], new_row, tensor[index_to_insert:]), dim=0) # tensor([[1, 2],
										 	  # 	    [3, 4], 
										 	  #         [7, 8],
										 	  #         [5, 6]])
```

<br/>
* 특정 행 삭제하기
{: .fs-5 .fw-700 .text-yellow-200 }

```python
# 초기 텐서 생성
x = torch.tensor([[1, 2], [3, 4], [5, 6]])

# 특정 행 삭제 (예: 두 번째 행)
index_to_delete = 1
result = torch.cat((tensor[:index_to_delete], tensor[index_to_delete + 1:]), dim=0) # tensor([[1, 2],
										     #         [5, 6]])
```

<br/>
* 응용
{: .fs-5 .fw-700 .text-yellow-200 }

```python
# depth image
x = torch.tensor([[1,  2,  3],
                  [4,  5,  6]])
y = torch.tensor([[7,  8,  9],
                  [10, 11, 12]])

# flatten
x_flattened = x.flatten(0) # tensor([1,  2,  3,  4,  5,  6])
y_flattened = y.flatten(0) # tensor([7,  8,  9, 10, 11, 12])

# concatenating
result = torch.cat([x_flattened, y_flattened], dim=0) # tensor([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12])

# reshape
result = result.view(2, 6) # tensor([[1,  2,  3,  4,  5,  6],
                           #         [7,  8,  9, 10, 11, 12]])

# 참고: concatenating + reshape의 결과는 torch.stack을 쓰면 된다
result_stack = torch.stack([x_flattened, y_flattened], dim=0) # tensor([[1,  2,  3,  4,  5,  6],
                                                              #         [7,  8,  9, 10, 11, 12]])
```
<br/>
---

참고  
PyTorch 문서: [https://pytorch.org/docs/stable/generated/torch.cat.html](https://pytorch.org/docs/stable/generated/torch.cat.html)   
