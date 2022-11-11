---
layout: default
title: roslaunch
parent: Ros
grand_parent: Robotics
permalink: /docs/robotics/ros/roslaunch/
---

ROSLAUNCH
{: .fs-7 .fw-700 }

roslaunch는 하나 이상의 노드를 실행하거나 실행 옵션을 설정하는 명령어이다. 하나의 패키지를 구동하는 것만으로도 여러개의 노드와 파라미터 서버가 실행되며 노드를 실행할 때 패키지의 파라미터나 노드 이름 변경, 노드 네임스페이스 설정, ROS_ROOT 및 ROS_PACKAGE_PATH 설정, 환경변수 변경 등의 옵션을 붙일 수도 있다. 실행 명령어는 'roslaunch [패키지명] [roslaunch 파일명]'이다.

```yaml
$ roslaunch package_name lauch_file
```
   
---

LAUNCH FILES
{: .fs-6 .fw-500 }
   
launch 파일은 roslaunch가 실행 노드를 설정할 때 사용하는 파일이다. XML 기반이며 태그별 옵션을 제공한다. 많은 ROS 패키지들이 launch 파일을 제공하고 있다.

