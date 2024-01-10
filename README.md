# Multi-Robot Graph SLAM using LIDAR Simulation Package

This repository contains a world and launch files for testing the the [Multi-Robot Graph SLAM Framework](https://github.com/aserbremen/Multi-Robot-Graph-SLAM) in a simulated environment using Gazebo (tested on ROS2 humble and Gazebo Fortress).

## Dependencies

This package depends on the following packages:

- [Gazebo Fortress](https://gazebosim.org/docs/fortress/install)
- [ros_gz](https://github.com/gazebosim/ros_gz/tree/humble)

```
sudo apt install ros-humble-ros-gz
```

## Usage

Launch the world in gazebo with the following command. Note that the first call to gazebo might take a while, since it has to download the models of the fortress.


```
ros2 launch mrg_slam_sim rubicon.launch.py
```

Spawn two robots equipped with LIDAR sensors and bridge the sensor data to ROS2 topics. Also start the teleop_joystick node to control the first robot atlas with a joystick (xbox). This terminal has to be kept active.

```
ros2 launch mrg_slam_sim dual_robot_sim.launch.py
```

Second robot bestla can be controlled with the keyboard using the following command:

```
/opt/ros/humble/lib/teleop_twist_keyboard/teleop_twist_keyboard --ros-args -r __node:=teleop_twist_keyboard_node -r /cmd_vel:=/bestla/cmd_vel
``` 

You can also control the first robot atlas with the keyboard using the following command:

```
/opt/ros/humble/lib/teleop_twist_keyboard/teleop_twist_keyboard --ros-args -r __node:=teleop_twist_keyboard_node_atlas -r /cmd_vel:=/atlas/cmd_vel
```

Now to test the Multi-Robot Graph SLAM Framework with the [mrg_slam](https://github.com/aserbremen/mrg_slam) package by running two instances of the SLAM algorithm in separate terminals:

```
ros2 launch mrg_slam mrg_slam.launch.py model_namespace:=atlas x:=-7.0 y:=0.0 z:=3.96 # terminal 1
ros2 launch mrg_slam mrg_slam.launch.py model_namespace:=bestla x:=0.0 y:=-15.0 z:=3.8 # terminal 2
```

Move the robots and visualize the results in rviz2:

```
rviz2 -d /path/to/mrg_slam/rviz/mrg_slam.rviz --ros-args -p use_sime_time:=true
```
