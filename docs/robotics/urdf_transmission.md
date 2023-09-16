---
layout: default
title: URDF Transmission
parent: Ros
grand_parent: Robotics
permalink: /docs/robotics/ros/urdf_transmission/
---

URDF Transmission
{: .fs-7 .fw-700 }

그리퍼 URDF를 작성하고 있었는데 난관... 까진 아니고 난간에 봉착... 기어를 어떻게 표현해야하지...   

Transmission은 URDF의 확장 기능으로, actuator와 joint 간의 관계를 설명하는 데 사용된다. 이를 통해 gear ratio 및 parallel linkages와 같은 개념을 모델링 할 수 있다. Transmissin은 effort/flow 변수를 변환하여 이들의 곱인 power가 일정하게 유지되도록 한다. 여러 actuators가 complex한 transmission을 통해 여러 joint에 연결 될 수 있다.   

다음은 transmission 사용 예이다.

```yaml
<transmission name="simple_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="foo_joint">
    <hardwareInterface>EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="foo_motor">
    <mechanicalReduction>50</mechanicalReduction>
    <hardwareInterface>EffortJointInterface</hardwareInterface>
   </actuator>
</transmission>
```

각각의 element들을 한 번 살펴보자.      
   
 
---

<transmiision> Attributes
{: .fs-6 .fw-700 }
   

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
<br>
      
{: .important-title}
> Note
>   
> .bag to .csv: rostopic ehco -b [bag file name] -p [topic name] > [csv file name]
>
> $ rostopic echo -b my_bag.bag -p /my_robot/cmd_vel > my_csv.csv

---

참고  
[ros wiki](http://wiki.ros.org/urdf/XML/Transmission)

