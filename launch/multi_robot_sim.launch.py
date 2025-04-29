import os
import subprocess

import sys
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import SetRemap, PushRosNamespace
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def values_to_strings_in_dict(dictionary):
    for key, value in dictionary.items():
        if isinstance(value, bool):
            dictionary[key] = str(value)
        elif isinstance(value, float):
            dictionary[key] = str(value)
        elif isinstance(value, dict):
            values_to_strings_in_dict(value)
    return dictionary


def get_gazebo_command():
    ign_result = subprocess.run(['which ign'], shell=True, capture_output=True, text=True)
    gz_result = subprocess.run(['which gz'], shell=True, capture_output=True, text=True)
    if ign_result.stdout.strip() != '':
        print('Detected Ignition Gazebo, trying to use ign command')
        return ign_result.stdout.strip()
    elif gz_result.stdout.strip() != '':
        print('Detected gz Gazebo, trying to use gz command')
        return gz_result.stdout.strip()


def generate_ros_gz_bridge_dict(sim_params):
    gz_command = get_gazebo_command()
    if 'ign' in gz_command:
        ign = True
    else:
        ign = False
    res = [{'gz_topic_name': 'clock',
           'ros_topic_name': 'clock',
            'gz_type_name': 'ignition.msgs.Clock' if ign else 'gz.msgs.Clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'direction': 'GZ_TO_ROS'}]
    for robot_name, ros_gz_bridge_ros_topic in zip(sim_params['robot_names'], sim_params['ros_gz_bridge_ros_topics']):
        for gz_topic, ros_topic in ros_gz_bridge_ros_topic.items():
            prefix = robot_name + '/' if robot_name != '' else ''
            gz_topic_name = prefix + gz_topic
            ros_topic_name = prefix + ros_topic
            topic_dict = {
                'gz_topic_name': gz_topic_name,
                'ros_topic_name': ros_topic_name,
                'direction': 'GZ_TO_ROS'  # default direction
            }
            if gz_topic == 'laser_scan/points':
                topic_dict.update({
                    'gz_type_name': 'ignition.msgs.PointCloudPacked' if ign else 'gz.msgs.PointCloudPacked',
                    'ros_type_name': 'sensor_msgs/msg/PointCloud2',
                })
            elif gz_topic == 'imu/data':
                topic_dict.update({
                    'gz_type_name': 'ignition.msgs.IMU' if ign else 'gz.msgs.IMU',
                    'ros_type_name': 'sensor_msgs/msg/Imu',
                })
            elif gz_topic == 'odom_ground_truth':
                topic_dict.update({
                    'gz_type_name': 'ignition.msgs.Odometry' if ign else 'gz.msgs.Odometry',
                    'ros_type_name': 'nav_msgs/msg/Odometry',
                })
            elif gz_topic == 'cmd_vel':
                topic_dict.update({
                    'gz_type_name': 'ignition.msgs.Twist' if ign else 'gz.msgs.Twist',
                    'ros_type_name': 'geometry_msgs/msg/Twist',
                    'direction': 'ROS_TO_GZ'  # Override direction for cmd_vel
                })
            res.append(topic_dict)
    return res


def generate_launch_description():
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')
    launch_description_list = []

    sim_config_path = os.path.join(mrg_slam_sim_share_dir, 'config', 'multi_robot_sim.yaml')
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)
        print(yaml.dump(sim_config, sort_keys=False, default_flow_style=False))
        teleop_joy_params = sim_config['teleop_joy']['ros__parameters']
        sim_params = sim_config['sim']['ros__parameters']

    if not (len(sim_params['robot_names']) == len(sim_params['init_poses']) == len(sim_params['ros_gz_bridge_ros_topics'])):
        print('The number of robot names, initial poses, and or ros_gz_bridge_ros_topics do not match, exiting...')
        exit(1)

    # Get the launch file for spawning a single robot
    spawn_robot_python_source = PythonLaunchDescriptionSource(
        os.path.join(mrg_slam_sim_share_dir, 'launch', 'spawn_robot.launch.py')
    )
    for robot, init_pose in zip(sim_params['robot_names'], sim_params['init_poses']):
        spawn_robot_params = {
            'robot_name': robot,
            **init_pose,  # unpacks x, y, z, roll, pitch, yaw
            'sdf_file': sim_params['sdf_file'],
            'use_sim_time': sim_params['use_sim_time'],
            'world': sim_params['world'],
        }
        values_to_strings_in_dict(spawn_robot_params)
        spawn_robot = IncludeLaunchDescription(
            spawn_robot_python_source,
            launch_arguments=spawn_robot_params.items(),
        )
        launch_description_list.append(spawn_robot)

    # Start the parameter bridge for communication between ROS2 and Ignition Gazebo
    ros_gz_config = os.path.join(mrg_slam_sim_share_dir, 'config', 'multi_robot_ros_gz_bridge.yaml')
    ros_gz_tmp_config = os.path.join(mrg_slam_sim_share_dir, 'config', 'multi_robot_ros_gz_bridge_tmp.yaml')
    os.system('cp ' + ros_gz_config + ' ' + ros_gz_tmp_config)
    ros_gz_bridge_dict = generate_ros_gz_bridge_dict(sim_params)
    print('Using the following ros_gz_bridge_dict:')
    print(ros_gz_bridge_dict)
    with open(ros_gz_tmp_config, 'w') as f:
        yaml.safe_dump(ros_gz_bridge_dict, f)

    ros_gz_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                        '--ros-args', '-p', 'config_file:=' + ros_gz_tmp_config], output='screen')
    launch_description_list.append(ros_gz_bridge)

    # Define the teleop node
    if teleop_joy_params['enable_teleop_joy']:
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
        launch_description_list.append(teleop_joy_node)

    print('If you want to control the robot with the keyboard, run the following command in a new terminal, e.g. for atlas')
    print('ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __node:=teleop_twist_keyboard_node_atlas -r /cmd_vel:=/atlas/cmd_vel')

    return LaunchDescription(launch_description_list)
