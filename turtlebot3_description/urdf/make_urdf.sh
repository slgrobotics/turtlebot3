#!/bin/sh

#Note: do "colcon build" in ~/turtlebot_create_ws first!

source ~/turtlebot_create_ws/install/setup.bash
xacro turtlebot3_create_1.urdf.xacro > turtlebot3_create_1.urdf

