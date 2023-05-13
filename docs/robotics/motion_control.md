---
layout: default
title: Motion Control
parent: Robot Control
grand_parent: Robotics
permalink: /docs/robotics/robot_control/motion_control/
---

Motion Control
{: .fs-7 .fw-700 }

motion control은 원하는 motion을 생성하기 위해서 torque를 어떻게 주어야할 지 계산하는 것이다. RL처럼 torque가  학습을 통해서 계산될수도 있고 운동방정식으로부터 직접 계산하는 등 다양한 방법이 있다. 크게 Joint space control과 Operation space control로 나뉜다. Joint space control은 desired joint angle이 정해질 때(어떤 경로로 각도들이 움직였으면 좋겠다) 이것을 만족하는 토크를 계산하는 것이다. Operation space control은 end effoctor의 정해졌을 때(end effector의 위치 등) roslaunch는 하나 이상의 노드를 실행하거나 실행 옵션을 설정하는 명령어이다. 하나의 패키지를 구동하는 것만으로도 여러개의 노드와 파라미터 서버가 실행되며 노드를 실행할 때 패키지의 파라미터나 노드 이름 변경, 노드 네임스페이스 설정, ROS_ROOT 및 ROS_PACKAGE_PATH 설정, 환경변수 변경 등의 옵션을 붙일 수도 있다. 실행 명령어는 'roslaunch [패키지명] [roslaunch 파일명]'이다.

```yaml
$ roslaunch package_name lauch_file
```
   
---

LAUNCH FILES
{: .fs-6 .fw-500 }
   
launch 파일은 roslaunch가 실행 노드를 설정할 때 사용하는 파일이다. XML 기반이며 태그별 옵션을 제공한다. 많은 ROS 패키지들이 launch 파일을 제공하고 있다.

