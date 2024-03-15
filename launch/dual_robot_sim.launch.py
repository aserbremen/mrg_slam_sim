import os

import sys
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def booleans_to_strings_in_dict(dictionary):
    for key, value in dictionary.items():
        if isinstance(value, bool):
            dictionary[key] = str(value)
    return dictionary


def generate_launch_description():
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')

    # Simple hack to get the world argument from the command line, see https://answers.ros.org/question/376816/how-to-pass-launch-args-dynamically-during-launch-time/
    print(f'Set the world (default marsyard2020), e.g. ros2 launch mrg_slam_sim dual_robot_sim.launch.py world:=rubicon')
    print(f'Available worlds: rubicon, marsyard2020')
    world = 'marsyard2020'
    for arg in sys.argv:
        if arg.startswith('world:='):
            world = arg.split(':=')[1]

    sim_config_path = os.path.join(mrg_slam_sim_share_dir, 'config', 'dual_robot_' + world + '_sim.yaml')
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)
        print(yaml.dump(sim_config, sort_keys=False, default_flow_style=False))
        teleop_joy_params = sim_config['teleop_joy']['ros__parameters']
        spawn_robot_params_1 = sim_config['spawn_robot_1']['ros__parameters']
        spawn_robot_params_2 = sim_config['spawn_robot_2']['ros__parameters']

    # Spawn the two robots
    spawn_robot_python_source = PythonLaunchDescriptionSource(
        os.path.join(mrg_slam_sim_share_dir, 'launch', 'spawn_robot.launch.py')
    )
    # transform all booleans to string, IncldueLaunchDescription does not support booleans
    booleans_to_strings_in_dict(spawn_robot_params_1)
    booleans_to_strings_in_dict(spawn_robot_params_2)
    spawn_robot_1 = IncludeLaunchDescription(
        spawn_robot_python_source,
        launch_arguments=spawn_robot_params_1.items(),
    )
    spawn_robot_2 = IncludeLaunchDescription(
        spawn_robot_python_source,
        launch_arguments=spawn_robot_params_2.items(),
    )

    # Start the parameter bridge for communication between ROS2 and Ignition Gazebo
    ros_gz_config = os.path.join(mrg_slam_sim_share_dir, 'config', 'dual_robot_ros_gz_bridge.yaml')
    ros_gz_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                        '--ros-args', '-p', 'config_file:=' + ros_gz_config], output='screen')

    if teleop_joy_params['enable_teleop_joy']:
        # Define the teleop node
        teleop_joy_node = GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst=teleop_joy_params['cmd_vel_topic']),
                SetRemap(src='/joy', dst=teleop_joy_params['joy_topic']),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([get_package_share_directory('teleop_twist_joy'), '/launch/teleop-launch.py']),
                    launch_arguments={'joy_config': teleop_joy_params['joy_config'], 'joy_dev': teleop_joy_params['joy_dev']}.items()
                )
            ]
        )

    print('If you want to control the robot with the keyboard, run the following commands in a new terminal')
    print('ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_twist_keyboard_node_atlas -r /cmd_vel:=/atlas/cmd_vel')
    print('ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_twist_keyboard_node_bestla -r /cmd_vel:=/bestla/cmd_vel')

    launch_list = [spawn_robot_1, spawn_robot_2, ros_gz_bridge]
    if teleop_joy_params['enable_teleop_joy']:
        launch_list.append(teleop_joy_node)
    return LaunchDescription(launch_list)
