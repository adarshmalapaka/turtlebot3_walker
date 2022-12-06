[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---

# TurtleBot3 Walker

## Overview 

Implementation of a simple walker algorithm where the TurtleBot3 moves in the world trying to avoid obstacles and collisions. 

## Assumptions
* OS: Ubuntu Linux Focal (20.04) 64-bit
* ROS2 Distro: Humble Hawksbill (atleast Galactic Geochelone)
* ROS2 Workspace name: ros2_ws 
* ROS2 Installation Directory: ros2_humble

## ROS2 Dependencies
* ```ament_cmake```
* ```rclcpp```
* ```geometry_msgs```
* ```sensor_msgs```
* ```gazebo_ros```
* ```turtlebot3```
* ```turtlebot3_msgs```
* ```dynamixel-sdk```
* ```turtlebot3_simulations```

## ROS2 Installation (source)

The following steps walkthrough the procedure to install the lastest LTS version of ROS2 (Humble) on an Ubuntu 20.04 machine, from source code. These steps can be found in [this link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

If your system is running Ubuntu Linux Jammy (22.04) 64-bit, you may skip to the binary installation of ROS2 Humble using 
[this link.](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

NOTE: The above procedure can take about 2+ hours to run. For a system with Intel Core i5 11th generation, it took nearly 6 hours.

### Environment Setup
```
. <path-to-ROS2-installation>/ros2_humble/install/local_setup.bash
```
### TurtleBot3 Installation

If you have a debian installation of ROS on your system, the steps to install TurtleBot3 packages are given [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) with ```foxy``` replaced with your corresponding ROS distribution.

For a source installation of ROS2, the steps to install Gazebo and Turtlebot3 packages can be found [here](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros) and [here](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html).

Only ROS2 ```galactic``` and beyond are supported as a certain argument for the ros2 bag command does not exist for ```foxy```.

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

export TURTLEBOT3_MODEL=waffle_pi

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo `/share/turtlebot3_gazebo/models/

cd <path-to-ROS2-workspace>/ros2_ws/src/turtlebot3_walker/results/bag_files   # To save the bag recording in the results directory
ros2 launch turtlebot3_walker tb3_walker.launch.py enable_recording:=false
```

where, ```enable_recording``` is an argument that enables/disables (set to true/false respectively) recording the ros2 bag of all topics except those of the ```/camera/*``` topics of TurtleBot3. The bag files are saved in the ```results/bag_files``` directory with the name ```walker_bag```.

Note: Sometimes upon launching the above file, the robot may not move despite the node working correctly by printing the computed Laser distances from the subscriber. In such a case, run the following in one terminal:

```
export TURTLEBOT3_MODEL=waffle_pi

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo `/share/turtlebot3_gazebo/models/

roslaunch turtlebot3_gazebo turtlebot3_world.launch.py
```

and run the ```walker``` node in another terminal as follows:

```
ros2 run turtlebot3_walker walker
```


### Handling ROS2 bag file

In the terminal sourced with your ROS2 installation and workspace, navigate to the ```results/bag_files``` directory and run the following:

```
ros2 bag info walker_bag
```

To play the recorded bag file (recorded for more than 20 seconds of simulation):

```
ros2 bag play walker_bag
```

## Results

### cpplint 
Change to the root directory of the package, ```/turtlebot3_walker```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/turtlebot3_walker/*.hpp > ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

### cppcheck
Change to the root directory of the package, ```/turtlebot3_walker```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/turtlebot3_walker/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.

