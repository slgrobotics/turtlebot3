# ROS2 packages for Turtlebot3 for Create 1 Base

### ROS Humble and Jazzy: use the default _ros2-devel_ branch!

Create 1 (Roomba 400) parameters: axel length: 0.262  wheel diameter: 0.066

This branch relies on the following physical robot setup: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1

(old version: https://github.com/slgrobotics/turtlebot_create)

*ROS Jazzy / Ubuntu 24.04* is not kind to ROBOTIS *turtlebot3_simulations* package. We use our own simulation for Dragger - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy

If using ROS Humble, also see: [https://github.com/slgrobotics/turtlebot3_simulations](https://github.com/slgrobotics/turtlebot3_simulations)

### Note: what runs on Raspberry Pi, and what on the Desktop machine:

My _Create 1 Turtlebot_ Raspberry Pi 3B runs three nodes ( https://github.com/slgrobotics/turtlebot_create/blob/main/RPi_Setup/launch/myturtle.py )

The Autonomy Labs *"create_driver"* node on RPi (https://github.com/slgrobotics/create_robot - you may use _foxy_ branch for Humble) isn't a ROS2 Turtlebot yet and requires an actual ROBOTIS *robot_state_publisher* node to run - which I run on the Desktop. See https://github.com/slgrobotics/turtlebot3/blob/ros2-devel/turtlebot3_bringup/launch/robot.launch.py

As the ROBOTIS code handled only "burger" and "waffle" bots, I had to fork their repository to add _Create 1_ URDF and introduce minor configuration changes.

**Note:** Instructions here relate to Desktop machine only.

## Build and run instructions (your Desktop machine)
```
mkdir -p ~/turtlebot_create_ws/src
cd ~/turtlebot_create_ws
```
Create a "repos" file:  ~/turtlebot_create_ws/turtlebot3.repos :

ROS Jazzy (Gazebo sim part would not build, so it is excluded):
```
repositories:
  turtlebot3/turtlebot3:
    type: git
    url: https://github.com/slgrobotics/turtlebot3.git
    version: ros2-devel
```
ROS Humble:
```
repositories:
  turtlebot3/turtlebot3:
    type: git
    url: https://github.com/slgrobotics/turtlebot3.git
    version: ros2-devel
  turtlebot3/turtlebot3_simulations:
    type: git
    url: https://github.com/slgrobotics/turtlebot3_simulations.git
    version: ros2-devel
```
Retrieve repositories and build:
```
cd ~/turtlebot_create_ws
vcs import src < turtlebot3.repos
colcon build

rosdep update
rosdep install --from-paths src --ignore-src -y
```
Add this to ~/.bashrc or otherwise set this environment variable:
```
export TURTLEBOT3_MODEL=create_1
```
Now you can run the robot:
```
source ~/turtlebot_create_ws/install/setup.bash
ros2 launch turtlebot3_bringup robot.launch.py
```
You can use any teleop package (keyboard or joystick):
```
ros2 run turtlebot3_teleop teleop_keyboard
```
Use Cartographer from binary Turtlebot 3 installation:
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
To save map ("my_map.pgm"):
```
ros2 run nav2_map_server map_saver_cli -f my_map
```
See https://github.com/ros-industrial/ros2_i_training/blob/main/workshop/source/_source/navigation/ROS2-Cartographer.md

## Running Nav2

You can run standard Turtlebot3 binaries, use "waffle" if needed:

```
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=False map:=/home/sergei/my_map.yaml

ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

The above will bring up full Nav2 stack and Rviz2

Note, that map->odom transformation is published by amcl which was launched as a part of nav2_bringup. But amcl needs an initial pose to start working.

In RViz there's a confusing sequence of clicks when you run Nav2 - to enable AMCL posting "map->odom" transform.

First click on Startup in Nav2 Panel in Rviz. Wait a minute, map should appear. Click on "2D Pose Estimate", wait till LIDAR readings appear (i.e. map->odom TF starts publishing).

If in doubt (i.e. not seeing "map" in TFs), you can always run static transform:
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

## ROBOTIS e-Manual for TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)

## Wiki for turtlebot3 Packages
- http://wiki.ros.org/turtlebot3 (metapackage)
- http://wiki.ros.org/turtlebot3_bringup
- http://wiki.ros.org/turtlebot3_description
- http://wiki.ros.org/turtlebot3_example
- http://wiki.ros.org/turtlebot3_navigation
- http://wiki.ros.org/turtlebot3_slam
- http://wiki.ros.org/turtlebot3_teleop

## Open Source related to TurtleBot3
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_deliver](https://github.com/ROBOTIS-GIT/turtlebot3_deliver)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [open_manipulator_with_tb3_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_msgs)
- [open_manipulator_with_tb3](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3)
- [open_manipulator_with_tb3_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_simulations)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [ROBOTIS e-Manual for Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3 ](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
