import os
import yaml
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
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
    'name': str,
    'sdf_file': str,
    'world': str,
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
    namespace = params['name']
    tree = ET.parse(sdf_path)
    for plugin in tree.findall('model'):
        plugin.attrib['name'] = namespace
    for plugin in tree.findall('model/plugin'):
        if plugin.attrib['name'] == 'ignition::gazebo::systems::OdometryPublisher':
            # Give the odometry publisher a unique topic name
            plugin.find('odom_topic').text = namespace + '/odom'

    tree.write(file_or_filename=sdf_path, encoding='utf-8', xml_declaration=True)


def launch_setup(context):
    print('spawning radio box')
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')
    config_file = os.path.join(mrg_slam_sim_share_dir, 'config', 'spawn_radio_box.yaml')
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)['spawn_radio_box']['ros__parameters']
    params = overwrite_yaml_params_from_cli(params, context.launch_configurations)
    print(yaml.dump(params, sort_keys=False, default_flow_style=False))

    name = params['name']
    sdf_path = os.path.join(mrg_slam_sim_share_dir, 'models', params['sdf_file'])
    # We create a temporary sdf file to namespace the cmd_vel and laser scan topics for use with multiple robots
    sdf_path_tmp = sdf_path.replace('.sdf', '_' + name + '.sdf')
    os.system(f'cp {sdf_path} {sdf_path_tmp}')

    namespace_sdf_file(sdf_path_tmp, params)

    ign_service_name = '/world/' + params['world'] + '/create'

    x, y, z = params['x'], params['y'], params['z']
    print(f'x: {x}, y: {y}, z: {z}')
    qx, qy, qz, qw = get_quaternion_from_euler(params['roll'], params['pitch'], params['yaw'])
    print(f'qx: {qx}, qy: {qy}, qz: {qz}, qw: {qw}')

    ign_request_args = 'sdf_filename: \"' + sdf_path_tmp + '\"' + ', name: \"' + params['name'] + \
        '\"' + f', pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}}}'

    cmd_string_list = ['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
                       'ignition.msgs.BooleanMsg', '--timeout', '10000', '--req', ign_request_args]
    command_string = ' '.join(cmd_string_list)
    print(command_string)

    process = ExecuteProcess(
        cmd=['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
             'ignition.msgs.Boolean', '--timeout', '10000', '--req', ign_request_args],
        output='screen')

    return [process]


def generate_launch_description():
    launch_description_list = []
    for param_name, _ in CLI_PARAM_MAPPING.items():
        launch_description_list.append(DeclareLaunchArgument(param_name, default_value='', description=''))
    launch_description_list.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_description_list)
