import os
import yaml
import xml.etree.ElementTree as ET
import copy

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


def configure_sdf_file(sdf_path, params):
    namespace = params['robot_name']
    prefix = namespace + '/' if namespace != '' else ''
    # Define a mapping of the topics that need to be namespaced or changed in the sdf file
    plugin_mappings = {
        # change plugins by name, only use defining name without prepending namespace in order to avoid ign, gz, ros confusion
        # e.g. 'DiffDrive' for ignition::gazebo::systems::DiffDrive and
        'DiffDrive': {
            'frame_id': lambda text: f'{prefix}{text}',
            'child_frame_id': namespace,
            'odom_topic': lambda text: f'{prefix}{text}',
            'tf_topic': lambda text: f'{prefix}{text}',
            'topic': lambda text: f'{prefix}{text}'
        },
        'OdometryPublisher': {
            'odom_frame': lambda text: f'{prefix}{text}',
            'robot_base_frame': lambda text: f'{prefix}{text}',
            'odom_topic': lambda text: f'{prefix}{text}',
            'tf_topic': lambda text: f'{prefix}{text}'
        }}

    sensor_mappings = {
        'velodyne': {
            'topic': f'{prefix}laser_scan',
            'ignition_frame_id': lambda text: f'{prefix}{text}'
        },
        'imu_sensor': {
            'always_on': '0',
            'topic': f'{prefix}imu/data',
            'ignition_frame_id': lambda text: f'{prefix}{text}'
        }
    }

    tree = ET.parse(sdf_path)

    if namespace != '':
        # Add namespace to the model name
        model = tree.find('.//model')
        model.attrib['name'] = namespace

    # Apply plugin mappings
    for plugin in tree.findall('.//plugin'):
        for element, change in plugin_mappings.items():
            if element in plugin.attrib['name']:
                # apply changes to the tree
                for tag, new_value in change.items():
                    elem = plugin.find(tag)
                    if elem is not None:
                        if callable(new_value):
                            elem.text = new_value(elem.text)
                        else:
                            elem.text = new_value

    # Apply sensor mappings
    for sensor in tree.findall('.//sensor'):
        for element, change in sensor_mappings.items():
            if element in sensor.attrib['name']:
                # apply changes to the tree
                for tag, new_value in change.items():
                    elem = sensor.find(tag)
                    if elem is not None:
                        if callable(new_value):
                            elem.text = new_value(elem.text)
                        else:
                            elem.text = new_value

    tree.write(file_or_filename=sdf_path, encoding='utf-8', xml_declaration=True)


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
    # We overwrite the final sdf file to configure it with the correct namespace and start pose for the odom frame
    sdf_path_final = sdf_path.replace('.sdf', '_' + robot_name + '.sdf')
    os.system(f'cp {sdf_path} {sdf_path_final}')
    configure_sdf_file(sdf_path_final, params)

    ign_service_name = '/world/' + params['world'] + '/create'

    x, y, z = params['x'], params['y'], params['z']
    qx, qy, qz, qw = get_quaternion_from_euler(params['roll'], params['pitch'], params['yaw'])

    ign_request_args = 'sdf_filename: \"' + sdf_path_final + '\"' + ', name: \"' + params['robot_name'] + \
        '\"' + f', pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}}}'

    cmd_string_list = ['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
                       'ignition.msgs.BooleanMsg', '--timeout', '10000', '--req', ign_request_args]

    print(f'executing command: {cmd_string_list}')
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
