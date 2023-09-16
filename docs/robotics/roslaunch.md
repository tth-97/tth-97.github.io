---
layout: default
title: roslaunch
parent: Ros
grand_parent: Robotics
permalink: /docs/robotics/ros/roslaunch/
---

roslaunch
{: .fs-7 .fw-700 }

roslaunch는 하나 이상의 노드를 실행하거나 실행 옵션을 설정하는 명령어이다. 하나의 패키지를 구동하는 것만으로도 여러개의 노드와 파라미터 서버가 실행된다. 또한 노드를 실행할 때 패키지의 파라미터나 노드 이름 변경, 노드 네임스페이스 설정, ROS_ROOT 및 ROS_PACKAGE_PATH 설정, 환경변수 변경 등의 옵션을 붙일 수도 있다. 실행 명령어는 'roslaunch <package-name> <launch-filename> [args]'이다. e.g.:   

```yaml
$ roslaunch my_package my_file.launch
```
   
위 명령을 통해 roslaunch는 my_package 안에 있는 my_file.launch를 찾아 실행한다.    

---

LAUNCH FILES
{: .fs-6 .fw-700 }
   
launch file은 roslaunch가 실행 노드를 설정할 때 사용하는 file이다. 다음은 launch file의 예시부터 살펴보자.   

```yaml
<launch>
    <arg name="wname" default="earth"/>
    <arg name="rname" default="laikago"/>
    <arg name="robot_path" value="(find $(arg rname)_description)"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find unitree_gazebo)/worlds/$(arg wname).world"/>
        <arg name="gui" value="$(arg gui)"/>
    </include>

    <!-- Load the URDF into the ROS Parameter Server -->
    <param name="robot_description"
           command="cat '$(find laikago_description)/urdf/laikago.urdf'"/>

    <!-- load the parameter unitree_controller -->
    <include file="$(find unitree_controller)/launch/set_ctrl.launch">
        <arg name="rname" value="$(arg rname)"/>
    </include>

</launch>
```   

위 예시에서 알 수 있듯이 launch file은 XML 기반이며 태그(< >)로 이루어져 있다. 각 태그들은 그에 맞는 옵션을 제공하며, 위에서부터 아래로 순차적으로 평가된다. launch파일을 작성한다는 것은 태그를 어떻게 설정하냐는 것이다. 따라서 각 태그들이 어떤 의미가 있는지 또 무엇을 어떻게 설정해야하는지 살펴볼 필요가 있다.

---

< launch > tag
{: .label .label-yellow }

The < launch > tag is the root element of any roslauch file. Its sole purpuse is to act as a containere for the other elements:   
* **node**: Launch a node. 노드 실행에 대한 태그. 패키지, 노드명, 실행명을 변경할 수 있다.
* **param**: Set a parameter on the _Parameter Server_. 파라미터 name, type, value 등을 설정한다.
* **remap**: Declare a name remapping. 노드 이름, 토픽 이름 등의 노드에서 사용 중인 ROS 변수의 이름을 변경할 수 있다.
* **machine**: Declare a machine to use for launching. 노드를 실행하는 PC의 이름,  address, ros-root, ros-package-path등을 설정할 수 있다.
* **rosparam**: Set ROS parameters for the launch using a _rosparam_ file. rosparam 명령어처럼 load, dump, delet 등 파라미터 정보를 확인 및 수정한다.
* **include**: Include other _roslaunch_ files. 다른 패키지나 같은 패키지에 속해 있는 다른 launch를 불러와 하나의 launch 파일처럼 실행할 수 있다.
* **env**: Specify an environment variable for launched nodes. 경로, IP 등의 환경변수를 절정한다.
* **test**: Launch a test node. 노드를 테스트할 때 사용한다. < node >와 비슷하지만 테스트에 사용할 수 있는 옵션들이 추가되어 있다.
* **arg**: Declare an argument. launch 파일 내에 변수를 정의할 수 있어서 실행할 때 파라미터를 변경시킬 수 있다.
* **group**: Group enclosed elements sharing a namespace or remap. 실행되는 노드를 그룹화할 때 사용한다. 
  
다음으로 각각의 element tag들을 자세히 살펴보자. 
       
--- 
    
< node > tag
{: .label .label-yellow }
   
< node > 태그는 launch하려는 ros node를 지정한다. 이것은 가장 중요한 기능인 node의 시작 및 종료를 지원하기 때문에 가장 일반적인 roslaunch 태그이다. 
   
**Examples**         
   
```yaml
<node name="listener1" pkg="rospy_tutorials" type="listener.py" args="--test" respawn="ture" />
```
Launches the "listener1" node using the listener.py executable from the rospy_tutorials package with the command-line argument --test. If the node dies, it will automatically be respawned.   

```yaml
<node name="bar1" pkg="foo_pkg" type="bar" args="$(find baz_pkg)/resources/map.pgm" />
```
Launches the bar node from the foo_pkg package. This example uses substitution arguments to pass in a portable reference to baz_pkg/resources/map.pgm.
   
**Attributes**   
   
* pkg="mypackage": Package of node.
* type="nodetype": Node type. There must be a corresponding executable with the same name.   
* name="nodename": Node name. Name cannot contain a namespace. Use the ns attribute instead.   
* args="arg1 arg2 arg3": Pass arguments to node.   
* respawn="true"(default: False): Restart the node automatically if it quits.
* respawn_delay="30"(default: 0): If respqwn is true, wait respawn_delay seconds after the node failure is detected before attempting restart.
* required="true": If node dies, kill entire roslaunch.
* ns="foo": start the node in the 'foo' namespace.
* output="log/screen": If 'screen', stdout/stderr from the node will be sent to the screen. If 'log', the stdout/stderr output will be sent to a log file in $ROS_HOME/log, and stderr will continue to be sent to screen. The default is 'log'.
* clear_params="true/false"
* cwd="ROS_HOME/node"
* launch-prefix="prefix arguments"
* if="true/false"   
   
**Elements**   
< node > 태그안에 다음과 같은 XML 태그를 사용할 수도 있다: < env >, < remap >, < rosparam >, < param >   
   
   
---
   
< param > tag   
{: .label .label-yellow }
   
< param > 는 Parameter Server에 설정할 parameter를 정의한다.

**Example**  
   
```yaml
<param name="publish_frequency" type="double" value="10.0" />
```
   
**Attributes**
   
< param > 태그를 구성하고 있는 attributes는 다음과 같은 것들이 있다:   
* name="namespace/name": Parameter name. Namespaces can be included in the parameter name, but globally specified names should be avoided.
* value="value": Defines the value of the parameter.
* type="str/int/double/bool/yaml": Specifies the type of the parameter. If you don't specify the type, roslaunch will attempt to automatically determine the type.
* command="$(find pkg-name)/exe '$(find pkg-name)/arg.txt'": The output of the command will be read and stored as a string. It is strongly recommended that you use the package-relative $(find)/file.txt syntax to specify file arguments. You should also quote file arguments using single quotes due to XML escaping requirements.   
* textfile="$(find pkg-name)/path/file.txt"
* binfile="$(find pkg-name)/path/file"
   
    
---

< remap > tag   
{: .label .label-yellow }   

< remap > tag 를 사용하면 (실제로는 /some_topic을 subscribe하거나 publish하고 있지만) ros가 /some_other_topic을 subscribe하거 publish하고 있다고 생각하도록 속일 수 있다. 

**Example**   

```yaml
<reemap from="/different_topic" to="/needed_topic" />
```   
   
Now, when this node subscribes to topic /different_topic, the remapping makes it actually subscribe to topic /needed_topic. So, anyone publishing to /needed_topic ends up getting their message to this new node as well!   
   
**Attributes**   

* from="original-name": Remapped topic: name of the ROS topic that you are remapping FROM.
* to="new-name": Target name: name of the ROS topic that you are pointing the from topic TO.
   
Remember: this means that if you remap FROM topic "A" TO topic "B", then whenever a node thinks it is subscribing to topic "A", it is actually subscribing to topic "B", so anyone publishing to topic "B" will end up getting their message to this node!
   
   
--- 

< include > tag
{: .label .label-yellow}   

< include > 태그를 사용하면 다른 roslaunch XML 파일을 현재 파일로 가져올 수 있다. < group > 및 < remap > 태그를 포함하여 문서의 현재 볌위 내에서 가져올 수 있다.

**Example**  
  
```yaml
<include file="$(find pkg-name)/path/filename.launch" />
```

Runs the launchfile "filename.launch" in the package "pkg-name"   
  
**Atrributes**  
   
* file="$(find pkg-name)/path/filename.xml": Name of file to inclues.
* ns="foo": Import the file relative to the 'foo' namespace
* clear_params="true/false" 
* pass_all_args="true/false"  

**Elements**   
< inclue > 태그안에 다음과 같은 XML 태그를 사용할 수도 있다: < env >, < arg >    
    
   
---

< arg > tag   
{: .label .label-yellow }   

< arg > 태그를 사용하면 command-line을 통해 전달되거나, < include > 태그를 통해 전달되거나, 상위 레벨 파일에 대한 선언된 값을 지정하여 재사용 가능하고 confiuarable한 launch file을 만들 수 있다. 이때 전달된는 args는 global 변수가 아니다. method의 local parameter와 마찬가지로 하나의 launch file에 한전된다. 또한 method 호출에서와 마찬가지로 arg values를 included file에 명시적으로 전달해야 한다.   

< arg > can be used in one of three ways:

```yaml
<arg name="foo" />
```   
Declares the existence of foo. foo must be passed in either as a command-line argument (if top-level) or via <include> passing (if included).
  

```yaml   
< arg name="foo" default="1" />
```  
Declares foo with a default value. foo can be overriden by command-line argument (if top-level) or via <include> passing (if included).
   
   
```yaml
<arg name="foo" value="bar" />
```   
Declares foo with constant value. The value for foo cannot be overridden. This usage enables internal parameterization of a launch file without exposing that parameterization at higher levels.   
   
   
**Examples**   
   
**case 1. Passing an argument to an included file**
    
*my_file.launch*
```yaml
<include file="included.launch">
  <!-- all vars that included.launch requires must be set -->
  <arg name="hoge" value="fuga" />
</include>
```   
   
   
*included.launch*
```yaml
<launch>
  <!-- declare arg to be passed in -->
  <arg name="hoge" />

  <!-- read value of arg -->
  <param name="param" value="$(arg hoge)"/>
</launch>
```   
   
**case 2. passing an argument via the command-line**  
    
roslaunch uses the same syntax as ROS remapping arguments to specify arg values.   

```yaml
$ roslaunch my_file.launch hoge:=my_value
$ roslaunch my_package my_file.launch hoge:=my_value
```     
    
**case 3. use with < param > tag**   
   
파라미터 설정인 < param >와 launch 파일 내의 변수인 < agr >를 이용하면 launch file을 실행할 때 외부에서 내부 변수를 변경할 수 있어서 노드내에서 사용하는 파라미터까지도 실행과 동시에 변경하는 것이 가능해져 매우 유용하다.   
      
```yaml
<launch>
  <arg name="update_period" default="10 />
  <param name="timing" value="$(arg update_period)"/>
</launch>
```   

   
**Attributes**
   
* name="arg_name": Name of argument.
* defult="default value" (optional): Default value of argument. Cannot be combined with _value_ attribute.
* value="value" (optional): Argument value. Cannot be combined with _default_ attribute.
* doc="description for this arg" (optional): Description of the argument. You could get this through --ros-args argument to the roslaunch command.

---

참고  
[ros wiki](http://wiki.ros.org/roslaunch)    
[ROS 로봇 프로그래밍](https://github.com/robotpilot/ros-seminar)

