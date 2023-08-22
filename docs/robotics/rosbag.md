---
layout: default
title: rosbag
parent: Ros
grand_parent: Robotics
permalink: /docs/robotics/ros/rosbag/
---

rosbag
{: .fs-7 .fw-700 }

bag은 ROS 메시지 데이터를 저장하기 위한 ROS의 파일 형식이다. .bag확장자를 가지며 다양한 tool을 사용하여 저장, 처리, 분석 및 시각화 될 수 있다. bag의 유용한 기능중 하나는 각종 메시지를 저장하고 필요할 때 이를 재생하여 이전 상황을 그대로 재현할 수 있는 것이다. 

rosbag은 bag을 생성, 재생, 압축 등을 하는 프로그램으로 다음과 같은 다양한 기능들을 가지고 있다:

* **record**: Record a bag file with the contents of specified topics.
* **info**: Summarize the contents of a bag file.
* **play**: Summarize the contents of a bag file.
* **check**: Determine whether a bag is playable in the current system, or if it can be migrated.
* **fix**: Repair the messages in a bag file so that it can be played in the current system.
* **filter**: Convert a bag file using Python expressions.
* **compress**: Compress one or more bag files.
* **decompress**: Decompress one or more bag files.
* **reindex**: Reindex one or more broken bag files.

 
---

rosbag record
{: .fs-6 .fw-700 }
   
rosbag record는 topic을 subscribe하고, 해당 topic에 publish된 모든 메시지를 .bag 파일에 작성한다. 즉 지정한 topic의 메시지를 .bag 파일에 기록하는 것이다. 아래와 같이 사용 중인 topic중에서 기록할 topic을 옵션으로 입력하여 bag 기록을 시작할 수 있다. 
   
```yaml
$ rostopic list
/rosout
/my_robot/cmd_vel
/my_robot/pose
``` 

```yaml
# rosbag record [topic name]
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
[ros wiki](http://wiki.ros.org/rosbag/Commandline)    
[ROS 로봇 프로그래밍](https://github.com/robotpilot/ros-seminar)

