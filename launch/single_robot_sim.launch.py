import os
import shutil
import sys
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


def booleans_to_strings_in_dict(dictionary):
    result = dictionary.copy()
    for key, value in dictionary.items():
        if isinstance(value, bool):
            result[key] = str(value)
    return result


def namespace_ros_gz_config(ros_gz_config, params):
    model_namespace = params['robot_name']
    print(f'params: {params}')
    # copy the ros_gz_config to a new file considering namespace and symlink
    ros_gz_config_namespaced = ros_gz_config.replace('.yaml', '_' + model_namespace + '.yaml')  # we return this
    ros_gz_config_real = os.path.realpath(ros_gz_config)
    # copy ros_gz_config to ros_gz_config_namespaced, follow symlinks
    shutil.copy(ros_gz_config_real, ros_gz_config_namespaced, follow_symlinks=True)

    # read the yaml file
    with open(ros_gz_config_namespaced, 'r') as f:
        ros_gz_yaml = yaml.safe_load(f)

    # add namespace to the topics
    to_remove = []
    for bridged_msg_dict in ros_gz_yaml:
        if bridged_msg_dict['ros_topic_name'] == 'clock':
            continue

        prefix = model_namespace + '/' if model_namespace != '' else ''
        if bridged_msg_dict['ros_topic_name'] == "tf" and params['publish_tf_to_ros'] == True:
            prefix = 'model/' + prefix
            bridged_msg_dict['gz_topic_name'] = prefix + bridged_msg_dict['gz_topic_name']
            continue
        elif bridged_msg_dict['ros_topic_name'] == "tf" and params['publish_tf_to_ros'] == False:
            # remove the tf messages from the ros_gz_yaml
            print(f'Removing tf from ros_gz_bridge_config')
            to_remove.append(bridged_msg_dict)

        bridged_msg_dict['ros_topic_name'] = prefix + bridged_msg_dict['ros_topic_name']
        bridged_msg_dict['gz_topic_name'] = prefix + bridged_msg_dict['gz_topic_name']

   # remove the msgs that are not needed
    for msg in to_remove:
        ros_gz_yaml.remove(msg)

    # write the yaml file with the added namespace
    print(f'Writing ros_gz_bridge_config to: {ros_gz_config_namespaced}')
    print(yaml.dump(ros_gz_yaml, sort_keys=False, default_flow_style=False))
    with open(ros_gz_config_namespaced, 'w') as f:
        yaml.dump(ros_gz_yaml, f, sort_keys=False, default_flow_style=False)

    return ros_gz_config_namespaced


def generate_launch_description():
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')

    sim_config_path = os.path.join(mrg_slam_sim_share_dir, 'config', 'single_robot_sim.yaml')
    with open(sim_config_path, 'r') as f:
        sim_config = yaml.safe_load(f)
        print(yaml.dump(sim_config, sort_keys=False, default_flow_style=False))
        teleop_joy_params = sim_config['teleop_joy']['ros__parameters']
        spawn_robot_params = sim_config['spawn_robot']['ros__parameters']

    model_namespace = spawn_robot_params['robot_name']

    # Spawn the two robots
    spawn_robot_python_source = PythonLaunchDescriptionSource(
        os.path.join(mrg_slam_sim_share_dir, 'launch', 'spawn_robot.launch.py')
    )
    # transform all booleans to string, IncldueLaunchDescription does not support booleans
    spawn_robot_string_params = booleans_to_strings_in_dict(spawn_robot_params)
    spawn_robot = IncludeLaunchDescription(
        spawn_robot_python_source,
        launch_arguments=spawn_robot_string_params.items(),
    )

    # Start the parameter bridge for communication between ROS2 and Ignition Gazebo
    ros_gz_config = os.path.join(mrg_slam_sim_share_dir, 'config', 'single_robot_ros_gz_bridge.yaml')
    if model_namespace != '':
        ros_gz_config = namespace_ros_gz_config(ros_gz_config, spawn_robot_params)
    ros_gz_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                        '--ros-args', '-p', 'config_file:=' + ros_gz_config], output='screen')

    if teleop_joy_params['enable_teleop_joy']:
        # Define the teleop node
        if model_namespace != '':
            teleop_joy_params['cmd_vel_topic'] = model_namespace + '/' + teleop_joy_params['cmd_vel_topic']
            teleop_joy_params['joy_topic'] = model_namespace + '/' + teleop_joy_params['joy_topic']
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

    launch_list = [spawn_robot, ros_gz_bridge]
    if teleop_joy_params['enable_teleop_joy']:
        launch_list.append(teleop_joy_node)
    return LaunchDescription(launch_list)
