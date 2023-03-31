# ROS2 packages for Turtlebot3 for Create 1 Base
## use ros2-devel branch!

Create 1 (Roomba 400) parameters: axel length: 0.262  wheel diameter: 0.066

This branch relies on the following physical robot setup: https://github.com/slgrobotics/turtlebot_create

Also see: [https://github.com/slgrobotics/turtlebot3_simulations.git](https://github.com/slgrobotics/turtlebot3_simulations/tree/ros2-devel)  - use ros2-devel branch!

### Note: what runs on Raspberry Pi, and what on the Desktop machine:

My Turtlebot RPi 3B runs three nodes ( https://github.com/slgrobotics/turtlebot_create/blob/main/RPi_Setup/launch/myturtle.py )

The Autonomy Labs *"create_driver"* node on RPi (https://github.com/slgrobotics/create_robot/tree/foxy) isn't a ROS2 Turtlebot yet and requires an actual ROBOTIS node to run - which can be run on the Desktop.

As the ROBOTIS code handled only "burger" and "waffle" bots, I had to fork their repository to add Create 1 URDF and introduce minor configuration changes.

Instructions here relate to Desktop machine only.

## Build and run instructions
```
mkdir -p ~/turtlebot_create_ws/src
cd ~/turtlebot_create_ws
```
Create a "repos" file:  ~/turtlebot_create_ws/turtlebot3.repos :
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
vcs import src<turtlebot3.repos
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
