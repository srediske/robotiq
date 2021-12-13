# Robotiq

This repository is based on the
[original Robotiq repository (which is no longer maintained)](https://github.com/ros-industrial/robotiq)
and the simulation parts has been updated to ROS Noetic and Gazebo 11.

The implemented models have been extended to work with both  
a position interface as well as a effort interface in Gazebo 11:  
`position_controllers/JointTrajectoryController`  
`effort_controllers/JointTrajectoryController`

#### TODO:

* Add and update controller files for real robot (+modbus etc.)

## Installation

#### Dependencies

This software requires a system setup with Robot Operating System (ROS).
It is recommended to use **Ubuntu 20.04** with
**[ROS Noetic](http://wiki.ros.org/noetic/Installation)** and
**[Catkin](https://catkin-tools.readthedocs.io/en/latest/installing.html)**.

#### Building

```bash
# source global ros
source /opt/ros/noetic/setup.bash

# create a catkin workspace
mkdir -p catkin_ws/src && cd catkin_ws

# clone the repository
git clone https://gitlab.zal.aero/stephan.rediske/robotiq

# install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin build

# activate the workspace (ie: source it)
source devel/setup.bash
```

## Usage

#### 3-Finger Adaptive Robot Gripper

For test purposes and as a stand-alone, use the launch file with a
fixed Gripper:

    roslaunch robotiq_gazebo robotiq_3f_bringup.launch
    
For integration into bigger environments (e.g. with Robot-arms) create a new
package with a high-level .xacro which combines all parts of the new robot.

Use `robotiq_3f_description/urdf/robotiq_3f_macro.xacro`, implement it
as child to the robot-arm and add the ros_control plugin,
see `robotiq_gazebo/urdf/robotiq_3f.xacro` as an example.

#### 2F-85 Gripper

For test purposes and as a stand-alone, use the launch file with a
fixed Gripper:

    roslaunch robotiq_gazebo robotiq_2f85_bringup.launch

For integration into bigger environments (e.g. with Robot-arms) create a new
package with a high-level .xacro which combines all parts of the new robot.

Use `robotiq_2f_description/urdf/robotiq_2f_macro.xacro`, implement it
as child to the robot-arm and add the ros_control plugin,
see `robotiq_gazebo/urdf/robotiq_2f85.xacro` as an example.

#### Note

For simplicity, an argument has been added to the top-level launch files
(`robotiq_gazebo/launch/robotiq_3f_bringup.launch` and
`robotiq_gazebo/launch/robotiq_2f85_bringup.launch`)
to switch between position and effort interface, just use the flag
`use_effort_controller:=false` to start with position interface.
By default it's true, so it starts with effort interface.
