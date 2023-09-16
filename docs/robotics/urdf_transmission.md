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

각각의 element들을 살펴보자.      
   
 
---

< type > tag
{: .label .label-yellow }

한 번만 작성하며, transmission type를 지정한다. 현재는 "transmission_interface/SimpleTransmission"만 지원하고 있다.   

아...
Isaac Gym에서 transmission 지원안하네... 당장은 알아야할 필요가 없어졌다. 여유가 될 때 이어서 작성해야겠다.....
(https://forums.developer.nvidia.com/t/are-urdf-transmission-elements-supported/177114)


---

참고  
[ros wiki](http://wiki.ros.org/urdf/XML/Transmission)

