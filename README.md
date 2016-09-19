cpr_mover
=========

ROS package to interface to the CPR Mover4 and Mover6 robot arms.

DOCUMENTATION --> /doc folder

Functionalities:
This ROS package allows to include the Commonplace Robotics Mover4 and Mover6 robots into an ROS environment. The main functionalities are:
* Establishing a robot control loop
* Connecting to the hardware robot arm using the USB-CAN adapter
* Publishing the joint states (6 joints + 2 gripper joints)
* Receiving joint velocites and commands like connect, enable, ...
* Receiving joint_trajectory_actions e.g. from MoveIt

This package should be used with an user interface to start with. A RViz plugin is availabe on github.com/CPR-Robots/cpr_rviz_plugin. This allows to jog the robot and to visualize it in RViz.


