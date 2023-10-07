---
layout: default
title: Dynamixel
parent: Ros
grand_parent: Robotics
permalink: /docs/robotics/ros/dynamixel/
---

Dynamixel
{: .fs-7 .fw-700 }

다이나믹셀은 ROBOTIS사에서 나온 actuator로, 통신 변환 디바이스인 U2D2를 경유하여 제어명령을 actuator에 전달하거나 OpenCR과 같은 임베디드 보드를 통해 제어가 가능하다. 이렇게 다양한 환경에서 제어하기 위하여 DynamixelSDK라는 이름으로 개발환경을 지원하는데, 특히 ROS 패키지를 지원하고 있어 ROS로 제어 가능하다!! ROBOTIS에서 공식적으로 제공하는 ROS패키지는 dynamixel_workbench로, 공식 DynamidelSDK를 이용하고 있고 GUI 툴을 통한 모터 설정, 위치/속도/토그 제어 등 다양하게 활용할 수 있다.


다이나믹과 U2D2를 사용해서 그리퍼 제어를 해야하므로.....(ㅎㅎ) ROS에서 어떻게 사용하는지 알아보자.   


---

Downloads
{: .fs-6 .fw-700 }

* Main packages
{: .fs-5 .fw-700 .text-blue-100 }

```yaml
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
```

* Dependent packages
{: .fs-5 .fw-700 .text-blue-100 }

```yaml
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

git으로 패키지 세개 다운로드 후 catkin_make

---

Device Setup
{: .fs-6 .fw-700 }

* Copy rules file
{: .fs-5 .fw-700 .text-blue-100 }

```yaml
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/dynamixel-workbench/master/99-dynamixel-workbench-cdc.rules
$ sudo cp ./99-dynamixel-workbench-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

* Check usb port
{: .fs-5 .fw-700 .text-blue-100 }

```yaml
$ ls /dev/tty*
```

이 때 /dev/ttyUSB0 가 있으면 U2D2를 사용할 준비를 마친 것이다.

---


---

참고  
[ros wiki](http://wiki.ros.org/urdf/XML/Transmission)

