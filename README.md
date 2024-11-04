# Automated Mobile Robot (AMR)
This project is an implementation of an automated mobile robot designed to perform autonomous navigation tasks. It utilizes ROS2, Gazebo for simulation, and custom control algorithms for precise maneuvering in various environments.

# Table of Contents
1) Introduction

2) Features

3) Directory Structure

4) Getting Started

# Introduction
This automated mobile robot is designed to autonomously navigate through a structured environment, avoiding obstacles and following predefined paths. The project uses ROS2 for control and communication, along with Gazebo for simulation. The robot's motion control is based on a differential drive configuration and it is capable of handling commands through publishing commands over /cmd_vel topic.

# Features
1) Autonomous navigation using ROS2 navigation stack.

2) Collision avoidance with LiDAR or other sensor inputs.

3) Simulation environment in Gazebo for testing algorithms.

4) Custom control algorithms for precise motor control.

5) Remote control and telemetry data feedback via ROS2 topics.

# Directory Structure
amr/

config/

├── carto_lds_2d.lua

├── display.rviz

├── map_1.pgm

├── map_1.yaml

├── nav2_params.yaml

└── navigation.rviz

launch/

├── bot.launch.py

├── mapping.launch.py

├── navigation.launch.py

├── occupancy_grid.launch.py

├── rplidar.launch.py

└── rsp.launch.py

meshes/

├── base_link.stl

├── left_wheel_1.stl

├── lidar_1.stl

└── right_wheel_1.stl

resource/

├── amr

└── esp32_code.ino

urdf/

├── diff_drive_plugin.xacro

├── lidar_plugin.xacro

├── materials.xacro

├── robot.xacro

└── robot.trans

worlds/

└── world_1

CMakeLists.txt

package.xml

setup.py

setup.cfg

# Getting Started
1) Clone the repository:-
   
   `git clone https://github.com/GradVizor/amr.git` 
   
2) Make sure to install required packages and dependencies:-
   
    a) Cartographer (for mapping) :- `$ sudo apt install ros-humble-cartographer`
   
    b) Navigation stack (to perform autonomous features) :- `sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup`
    
    c) rplidar_ros (to get the data from a real rplidar) :- `$ sudo apt install ros-humble-rplidar-ros`
    
3) Build your project:
   
Ensure your build system includes the necessary source files and header paths. 

