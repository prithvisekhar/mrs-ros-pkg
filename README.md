mrs-ros-pkg [Modular Robot ROS package]
===========

ROS packages developed by the [Group of Robots and Intelligent Machines](http://www.romin.upm.es/) from the [Universidad Politécnica de Madrid](http://www.upm.es/internacional). This group is part of the [Centre for Automation and Robotics](http://www.car.upm-csic.es/) (CAR UPM-CSIC). On going development continues in the hydro-devel branch.

**Maintainer:** Prithvi Sekhar Pagala, 

### Documentation

  * See the installation instructions below.
  * This repository.
  * Throughout the various files in the packages.
  * For questions, please use [http://answers.ros.org](http://answers.ros.org)

### Build Status




## Installation

### Basic Requirements

  1. Install [ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (**Desktop Install** Recommended)
  2. Install [Gazebo 2.0](http://gazebosim.org/wiki/2.0/install)

### Repository Installation

Go to your ROS working directory. e.g.
```
cd ~/catkin_ws/src
``` 
Use the `wstool` to install the repository [http://wiki.ros.org/wstool] (http://wiki.ros.org/wstool)
```
wstool init .
wstool merge https://github.com/prithvisekhar/mrs-ros-pkg.git/mrs.rosinstall
wstool update
``` 
Install any missing dependencies using rosdep:
```
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro hydro
``` 
Now compile your ROS workspace. e.g.
```
cd ~/catkin_ws && catkin_make
``` 

### Testing Installation

Be sure to always source the appropriate ROS setup file, which for Hydro is done like so:
```
source /opt/ros/hydro/setup.bash
``` 
You might want to add that line to your `~/.bashrc`

Try any of the `.launch` files in the `mrs_gazebo` package: (e.g. `*.launch`)
```
roslaunch mrs_gazebo *.launch
``` 

## Changelog
