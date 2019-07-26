[![Udacity - Robotics NanoDegree Program](https://camo.githubusercontent.com/0f51a1a655e13e62c95e95dfe5850bf2c20b1dd6/68747470733a2f2f73332d75732d776573742d312e616d617a6f6e6177732e636f6d2f756461636974792d726f626f746963732f45787472612b496d616765732f526f626f4e445f666c61672e706e67)](https://www.udacity.com/robotics)

# udacity-robond-p4

**Map My World - Project 4 of the Udacity Robotics Software Engineer Nanodegree**

![Alt text](./docs/p2.png)

## Description

This project contains:

1. A two-wheeled robot model (URDF) which is equiped with a RGBD camera and a lidar.
2. A customized world supported by Gazebo. 
3. Three ROS packages: the `localization` , the `my_robot`  and `mapping`, which enable the robot to localize itself using Adaptive Monte Carlo Localization (http://wiki.ros.org/amcl) method as well as perform mapping in the environment and detect closed loops using `RTAB-MAP` ROS package. 



## Run 

`roslaunch my_robot world.launch`

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

`roslaunch mapping mapping.launch`



## Behavior

 <img src = "./docs/p1.png">



## Complete Map

Check the link below for a complete map database  `rtabmap.db`

https://drive.google.com/file/d/1H7ilPpPP5pIrL6AC4G7XPBvyUIotqjqL/view?usp=sharing

