---
layout: default
title: cheatsheet
parent: Ros
grand_parent: Robotics
nav_order: 4
permalink: /docs/robotics/ros/cheatsheet/
---

Cheatsheet
{: .fs-7 .fw-700 }

Ros melodic 명령어 Cheatsheet

---

Filesystem Management Tools
{: .fs-6 .fw-700 }

* rospack
{: .fs-5 .fw-700 .text-blue-100 }

rospack은 ROS 패키지와 관련된 정보를 표시하는 명령어로 find, list, depends-on, depends, profile 등의 옵션을 사용할 수 있다.  

다음 예제와 같이 **rospack find** 명렁어 다음에 패키지 이름을 지정하면 해당 패키지의 저장 위치가 표시된다. 

```yaml
# rospack find [package name]
rospack find my_package
/home/kang/catkin_ws/src/my_package
```

**rospack list**는 PC에 있는 모든 패키지를 표시하는 명령어이다. rospack list 명령어와 리눅스의 검색 명령어인 grep을 조합하면 원하는 패키지만 관련하여 찾을 수 있다. 


```yaml
$ rospack list
rqt_topic /opt/ros/melodic/share/rqt_topic
rqt_virtual_joy /opt/ros/melodic/share/rqt_virtual_joy
rqt_web /opt/ros/melodic/share/rqt_web
rviz /opt/ros/melodic/share/rviz
rviz_plugin_tutorials /opt/ros/melodic/share/rviz_plugin_tutorials
rviz_python_tutorial /opt/ros/melodic/share/rviz_python_tutorial
self_test /opt/ros/melodic/share/self_test
```

```yaml
$ rospack list | grep rviz
rviz /opt/ros/melodic/share/rviz
rviz_plugin_tutorials /opt/ros/melodic/share/rviz_plugin_tutorials
rviz_python_tutorial /opt/ros/melodic/share/rviz_python_tutorial
```
<br>
* roscd
{: .fs-5 .fw-700 .text-blue-100 }

roscd는 지정한 ROS 패키지가 저장된 디렉터리로 바로 이동하는 명령어이다. **roscd** 명령어 이후 파라미터에 패키지 이름을 적으면 된다.
   
```yaml
# roscd [package name]
$ roscd my_package
``` 

```yaml
$ pwd
/home/kang/catkin_ws/src/my_package
```
<br>
* rosbag
{: .fs-5 .fw-700 .text-blue-100 }

```yaml
# rosbag record [topic name]
$ rosbag record /my_robot/cmd_vel /my_robot/pose
```

특정 토픽이 아닌 모든 토픽을 동시에 기록하고 싶다면 '-a' 옵션을 붙인다.

```yaml
$ rosbag record -a
```
---

참고  
ROS 깃허브: [https://github.com/ros/cheatsheet/releases](https://github.com/ros/cheatsheet/releases)    
ROS 로봇 프로그래밍 깃허브: [https://github.com/robotpilot/ros-seminar](https://github.com/robotpilot/ros-seminar)

