---
layout: default
title: ??
parent: Ros
grand_parent: Robotics
nav_order: 5
permalink: /docs/robotics/ros/ros_programming/
---

??
{: .fs-7 .fw-700 }

최소 실행 단위인 노드는 메시지 통신을 통해 노드 간의 입출력 메시지를 주고받는다. 이때 토픽, 서비스, 액션, 파라미터를 사용한다.
ROS 메시지 통신에서 사용되는 publisher와 subscriber는 각각 송신과 수신을 담당한다.

---

1. Package 분석
{: .fs-6 .fw-700 }
   
Package를 생성하면 ~/catkin_ws/src 폴도에 생성한 패키지 폴더가 생기고, 이 패키지 폴더에는 ROS 패키지가 갖추어야 할 기본 폴더 그리고 CMakeLists.txt와 package.xml 파일이 생성된다.

```yaml
$ cd ~/catkin_ws/src/my_package
$ ls
include 	# 헤더 파일 폴더
src		# 소스 코드 폴더
CMakeLists.txt  # 빌드 설정 파일
package.xml 	# 패키지 설정파일
```

package.xml
{: .fs-5 .fw-700 .text-blue-100 }

ROS의 필수 설정 파일 중 하나인 **package.xml**은 패키지 정보가 담긴 XML 파일로서 패키지 이름, 저작자, 라이선스, 의존성 패키지 등을 기술하고 있다. 
 
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

roscd
{: .fs-5 .fw-700 .text-blue-100 }

**roscd**는 지정한 ROS 패키지가 저장된 디렉터리로 바로 이동하는 명령어이다. roscd 명령어 이후 파라미터에 패키지 이름을 적으면 된다.
   
```yaml
# roscd [package name]
$ roscd my_package
``` 

```yaml
$ pwd
/home/kang/catkin_ws/src/my_package
```


# rosbag record [topic name]

```yaml
$ rosbag record /my_robot/cmd_vel /my_robot/pose
```

특정 토픽이 아닌 모든 토픽을 동시에 기록하고 싶다면 '-a' 옵션을 붙인다.

```yaml
$ rosbag record -a
```

--- 
    
{: .important-title}
> Note
>  
> bag to csv
> : ropstopic ehco -b [bag file name] -p [topic name] > [csv file name]
> 
> $ rostopic echo -b my_bag.bag -p /my_robot/cmd_vel > my_csv.csv

---

참고  
ROS 깃허브: [https://github.com/ros/cheatsheet/releases](https://github.com/ros/cheatsheet/releases)    
ROS 로봇 프로그래밍 깃허브: [https://github.com/robotpilot/ros-seminar](https://github.com/robotpilot/ros-seminar)

