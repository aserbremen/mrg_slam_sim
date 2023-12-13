import os

import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mrg_sim_share_dir = get_package_share_directory('mrg_sim')

    sim_config_path = os.path.join(mrg_sim_share_dir, 'config', 'dual_robot_sim.yaml')
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)
        print(yaml.dump(sim_config, sort_keys=False, default_flow_style=False))
        teleop_joy_params = sim_config['teleop_joy']['ros__parameters']
        spawn_robot_params_1 = sim_config['spawn_robot_1']['ros__parameters']
        spawn_robot_params_2 = sim_config['spawn_robot_2']['ros__parameters']

    # Spawn the two robots
    spawn_robot_python_source = PythonLaunchDescriptionSource(
        os.path.join(mrg_sim_share_dir, 'launch', 'spawn_robot.launch.py')
    )
    spawn_robot_1 = IncludeLaunchDescription(
        spawn_robot_python_source,
        launch_arguments=spawn_robot_params_1.items(),
    )
    spawn_robot_2 = IncludeLaunchDescription(
        spawn_robot_python_source,
        launch_arguments=spawn_robot_params_2.items(),
    )

    # Start the paraemter bridge for communication between ROS and Ignition Gazebo
    ros_gz_config = os.path.join(mrg_sim_share_dir, 'config', 'dual_robot_ros_gz_bridge.yaml')
    ros_gz_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                        '--ros-args', '-p', 'config_file:=' + ros_gz_config], output='screen')

    if teleop_joy_params['enable_teleop']:
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
    # The other robots have to be controlled with the keyboard teleop node in separate terminals

    launch_list = [spawn_robot_1, spawn_robot_2, ros_gz_bridge]
    if teleop_joy_params['enable_teleop']:
        launch_list.append(teleop_joy_node)
    return LaunchDescription(launch_list)
