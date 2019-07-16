[![Udacity - Robotics NanoDegree Program](https://camo.githubusercontent.com/0f51a1a655e13e62c95e95dfe5850bf2c20b1dd6/68747470733a2f2f73332d75732d776573742d312e616d617a6f6e6177732e636f6d2f756461636974792d726f626f746963732f45787472612b496d616765732f526f626f4e445f666c61672e706e67)](https://www.udacity.com/robotics)

# udacity-robond-p3

**Where Am I? - Project 3 of the Udacity Robotics Software Engineer Nanodegree**

![Alt text](./docs/profile.png)

## Description

This project contains:

1. A two-wheeled robot model (URDF) which is equiped with a camera and a lidar.
2. A customized world supported by Gazebo. 
3. Two ROS packages: the `localization` and the `my_robot` , which enable the robot to localize itself using Adaptive Monte Carlo Localization (http://wiki.ros.org/amcl) method.



## Run 

`roslaunch my_robot world.launch`

`roslaunch localization amcl.launch`

- To make the robot localize itself, you need to tele-operate the robot to move from starting position (bottom-right corner) to another end of the house (top-right corner).

## Behavior

 <img src = "./docs/loc1.png">



 <img src = "./docs/loc2.png">

