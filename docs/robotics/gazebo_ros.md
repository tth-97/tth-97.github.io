---
layout: default
title: gazebo_ros
parent: Ros
grand_parent: Robotics
permalink: /docs/robotics/ros/gazebo_ros/
---

gazebo_ros
{: .fs-7 .fw-700 }


---

spawn_model
{: .fs-6 .fw-500 }

```
Singularity> rosrun gazebo_ros spawn_model -h  
usage: spawn_model [-h] (-urdf | -sdf)  
(-file FILE_NAME | -param PARAM_NAME | -database MODEL_NAME | -stdin)  
                   -model MODEL_NAME [-reference_frame REFERENCE_FRAME]  
                   [-gazebo_namespace GAZEBO_NAMESPACE]   
                   [-robot_namespace ROBOT_NAMESPACE] [-unpause]    
                   [-wait MODEL_NAME] [-x X] [-y Y] [-z Z] [-R R] [-P P]    
                   [-Y Y] [-J JOINT_NAME JOINT_POSITION] [-package_to_model]    
                   [-b]    
                   
                   
Spawn a model in gazebo using the ROS API  


optional arguments:  
　-h, --help            show this help message and exit  
　-urdf                 Incoming xml is in urdf format  
  -sdf                  Incoming xml is in sdf format  
  -file FILE_NAME       Load model xml from file  
  -param PARAM_NAME     Load model xml from ROS parameter  
  -database MODEL_NAME  Load model XML from specified model in Gazebo Model  
                        Database  
  -stdin                Load model from stdin  
  -model MODEL_NAME     Name of model to spawn  
  -reference_frame REFERENCE_FRAME  
                        Name of the model/body where initial pose is defined.  
                        If left empty or specified as "world", gazebo world  
                        frame is used  
  -gazebo_namespace GAZEBO_NAMESPACE  
                        ROS namespace of gazebo offered ROS interfaces.  
                        Defaults to /gazebo/  
  -robot_namespace ROBOT_NAMESPACE  
                        change ROS namespace of gazebo-plugins  
  -unpause              !!!Experimental!!! unpause physics after spawning  
                        model  
  -wait MODEL_NAME      !!!Experimental!!! wait for model to exist  
  -x X                  x component of initial position, meters  
  -y Y                  y component of initial position, meters  
  -z Z                  z component of initial position, meters  
  -R R                  roll angle of initial orientation, radians  
  -P P                  pitch angle of initial orientation, radians  
  -Y Y                  yaw angle of initial orientation, radians   
  -J JOINT_NAME JOINT_POSITION  
                        initialize the specified joint at the specified  
                        position  
  -package_to_model     convert urdf <mesh filename="package://..." to <mesh  
                        filename="model://..."  
  -b                    bond to gazebo and delete the model when this program  
                        is interrupted  
```
