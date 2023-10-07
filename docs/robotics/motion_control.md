---
layout: default
title: Motion Control
parent: Robot Control
grand_parent: Robotics
permalink: /docs/robotics/robot_control/motion_control/
---

Motion Control
{: .fs-7 .fw-700 }

motion control은 원하는 motion을 생성하기 위해서 torque를 어떻게 주어야할 지 계산하는 것이다. RL처럼 torque가  학습을 통해서 계산될수도 있고 운동방정식으로부터 직접 계산하는 등 다양한 방법이 있다. 크게 Joint space control과 Operation space control로 나뉜다. Joint space control은 desired joint angle이 정해질 때(어떤 경로로 각도들이 움직였으면 좋겠다) 이것을 만족하는 토크를 계산하는 것이다. Operation space control은 end effoctor의 정해졌을 때(end effector의 위치 등) 
   
---



