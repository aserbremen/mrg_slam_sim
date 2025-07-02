import os
import yaml
import xml.etree.ElementTree as ET
import copy
import subprocess

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


# Parameter type mapping to infer the correct data type from the cli string
CLI_PARAM_MAPPING = {
    'robot_name': str,
    'world': str,
    'sdf_file': str,
    'use_imu': bool,
    'x': float,
    'y': float,
    'z': float,
    'roll': float,
    'pitch': float,
    'yaw': float,
}


def overwrite_yaml_params_from_cli(yaml_params, cli_params):
    print('Overwriting yaml parameters with command line parameters')
    for key, value in cli_params.items():
        if key in yaml_params and value != '':
            # Since all parameters from cli in ROS2 are strings, we need to infer the correct data type
            yaml_params[key] = CLI_PARAM_MAPPING[key](value)
            # Overwrite the boolean values since they are not correctly parsed, non empty strings are always True
            if value == 'true' or value == 'True':
                yaml_params[key] = True
            elif value == 'false' or value == 'False':
                yaml_params[key] = False
    return yaml_params


def namespace_sdf_file(sdf_path, params):
    namespace = params['robot_name']
    tree = ET.parse(sdf_path)
    for plugin in tree.findall('model/plugin'):
        # cmd_vel unique name
        topic = plugin.find('topic')
        if topic is not None and topic.text == 'cmd_vel':
            plugin.find('topic').text = namespace + '/cmd_vel'
        # Give robot_base_frame (Odometry ground) a unique topic name
        robot_base_frame = plugin.find('robot_base_frame')
        if robot_base_frame is not None and robot_base_frame.text == 'base_link':
            plugin.find('robot_base_frame').text = namespace + '/base_link'
        # Give a unique topic name for odometry ground truth
        odom_topic = plugin.find('odom_topic')
        if odom_topic is not None and odom_topic.text == 'odom_ground_truth':
            plugin.find('odom_topic').text = namespace + '/odom_ground_truth'
    for sensor in tree.findall('model/link/sensor'):
        if sensor.attrib['name'] == 'front_laser':
            # Give the laser scan points a unique topic name
            sensor.find('topic').text = namespace + '/laser_scan'
            # Create the ignition frame id for the laser scan and make it unique
            sensor.find('gz_frame_id').text = namespace + '/' + sensor.find('gz_frame_id').text
        if sensor.attrib['name'] == 'imu_sensor' and params['use_imu']:
            # Set always on for the imu sensor to true
            sensor.find('always_on').text = '0'
            # Give the imu sensor a unique topic name
            sensor.find('topic').text = namespace + '/imu/data'
            # Create the ignition frame id for the imu sensor and make it unique
            sensor.find('gz_frame_id').text = namespace + '/' + sensor.find('gz_frame_id').text

    tree.write(file_or_filename=sdf_path, encoding='utf-8', xml_declaration=True)


def get_gazebo_command():
    ign_result = subprocess.run(['which ign'], shell=True, capture_output=True, text=True)
    gz_result = subprocess.run(['which gz'], shell=True, capture_output=True, text=True)
    if ign_result.stdout.strip() != '':
        print('Detected Ignition Gazebo, trying to use ign command')
        return ign_result.stdout.strip()
    elif gz_result.stdout.strip() != '':
        print('Detected gz Gazebo, trying to use gz command')
        return gz_result.stdout.strip()

def launch_setup(context):
    print('launching robot')
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')
    config_file = os.path.join(mrg_slam_sim_share_dir, 'config', 'spawn_robot.yaml')
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)['spawn_robot']['ros__parameters']
    params = overwrite_yaml_params_from_cli(params, context.launch_configurations)
    # we only overwrite non empty parameters, however robot_name can be empty if no namespace is desired
    robot_name = context.launch_configurations['robot_name']
    params['robot_name'] = robot_name
    print(yaml.dump(params, sort_keys=False, default_flow_style=False))

    sdf_path = os.path.join(mrg_slam_sim_share_dir, 'models', params['sdf_file'])
    sdf_path_final = copy.deepcopy(sdf_path)
    if robot_name != '':
        # We overwrite the final sdf file to namespace all relevant topics
        sdf_path_final = sdf_path.replace('.sdf', '_' + robot_name + '.sdf')
        os.system(f'cp {sdf_path} {sdf_path_final}')
        namespace_sdf_file(sdf_path_final, params)

    x, y, z = params['x'], params['y'], params['z']
    qx, qy, qz, qw = get_quaternion_from_euler(params['roll'], params['pitch'], params['yaw'])
    gazebo_command = get_gazebo_command()
    if 'ign' in gazebo_command:
        gazebo_service_name = '/world/' + params['world'] + '/create'
        gazebo_request_args = 'sdf_filename: \"' + sdf_path_final + '\"' + ', name: \"' + params['robot_name'] + \
            '\"' + f', pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}}}'
        cmd_string_list = [gazebo_command, 'service', '-s', gazebo_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
                           'ignition.msgs.Boolean', '--timeout', '10000', '--req', gazebo_request_args]
        print(f'executing command: {cmd_string_list}')
        process = ExecuteProcess(cmd=cmd_string_list, output='screen')
    elif 'gz' in gazebo_command:
        process = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                       'launch', 'gz_spawn_model.launch.py')),
            launch_arguments={
                'world': params['world'],
                'file': sdf_path_final,
                'entity_name': params['robot_name'],
                'x': str(x), 'y': str(y), 'z': str(z),
                'roll': str(params['roll']), 'pitch': str(params['pitch']), 'yaw': str(params['yaw'])
            }.items()
        )
    else:
        print('Could not find gazebo command [gz or ign], exiting')
        exit(1)

    return [process]


def generate_launch_description():
    launch_description_list = []
    for param_name, _ in CLI_PARAM_MAPPING.items():
        launch_description_list.append(DeclareLaunchArgument(param_name, default_value='', description=''))
    launch_description_list.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description_list)
