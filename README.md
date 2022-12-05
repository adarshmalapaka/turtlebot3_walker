[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# TurtleBot3 Walker

## Overview 

Implementation of a simple walker algorithm where the TurtleBot3 moves in the world trying to avoid obstacles and collisions. 

## Assumptions
* OS: Ubuntu Linux Focal (20.04) 64-bit
* ROS2 Distro: Humble Hawksbill (built from source)
* ROS2 Workspace name: ros2_ws 
* ROS2 Installation Directory: ros2_humble

## ROS2 Dependencies
* ```ament_cmake```
* ```rclcpp```
* ```geometry_msgs```
* ```sensor_msgs```

## ROS2 Installation (source)

The following steps walkthrough the procedure to install the lastest LTS version of ROS2 (Humble) on an Ubuntu 20.04 machine, from source code. These steps can be found in [this link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

If your system is running Ubuntu Linux Jammy (22.04) 64-bit, you may skip to the binary installation of ROS2 Humble using 
[this link.](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

NOTE: The above procedure can take about 2+ hours to run. For a system with Intel Core i5 11th generation, it took nearly 6 hours.

### Environment Setup
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
```

### Clang
```
sudo apt install clang
export CC=clang
export CXX=clang++
colcon build --cmake-force-configure
```

### ROS2 Workspace
Here, an overlay workspace on top of the underlay installation workspace shall be created to place the custom-defined ROS2 packages. 
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
mkdir -p <path-to-ROS2-workspace>/ros2_ws/src
cd <path-to-ROS2-workspace>/ros2_ws/src
```
Source the 'underlay' installation workspace followed by the 'overlay',
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
```

## Build Instructions
```
cd <path-to-ROS2-workspace>/ros2_ws/src
git clone https://github.com/adarshmalapaka/turtlebot3_walker.git
cd ..  
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select turtlebot3_walker
```

## Run Instructions

### Simulation

In a terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 launch turtlebot3_walker tb3_walker.launch.py
```