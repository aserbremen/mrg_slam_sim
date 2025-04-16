import os
import yaml
import xml.etree.ElementTree as ET
import subprocess

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
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


def get_gazebo_command():
    ign_result = subprocess.run(['which ign'], shell=True, capture_output=True, text=True)
    gz_result = subprocess.run(['which gz'], shell=True, capture_output=True, text=True)
    if ign_result.stdout.strip() != '':
        print('Detected Ignition Gazebo, trying to use ign command')
        return ign_result.stdout.strip()
    elif gz_result.stdout.strip() != '':
        print('Detected gz Gazebo, trying to use gz command')
        return gz_result.stdout.strip()
    

def namespace_sdf_file(sdf_path, namespace):
    tree = ET.parse(sdf_path)
    for plugin in tree.findall('model'):
        plugin.attrib['name'] = namespace
    for plugin in tree.findall('model/plugin'):
        if plugin.attrib['name'] == 'ignition::gazebo::systems::OdometryPublisher' or \
           plugin.attrib['name'] == 'gz::sim::systems::OdometryPublisher':
            # Give the odometry publisher a unique topic name
            plugin.find('odom_topic').text = namespace + '/odom_ground_truth'
            plugin.find('robot_base_frame').text = namespace + '/base_link'

    tree.write(file_or_filename=sdf_path, encoding='utf-8', xml_declaration=True)


def ign_spawn_model_process(ign_service_name, sdf_path, model_name, x, y, z, qx, qy, qz, qw):
    ign_request_args = 'sdf_filename: \"' + sdf_path + '\"' + ', name: \"' + model_name + \
        '\"' + f', pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}}}'

    process = ExecuteProcess(
        cmd=['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
             'ignition.msgs.Boolean', '--timeout', '10000', '--req', ign_request_args],
        output='screen')
    return process


def generate_launch_description():
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')
    config_file = os.path.join(mrg_slam_sim_share_dir, 'config', 'ranging_sim.yaml')
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)['ranging_sim']['ros__parameters']
        yaml.dump(params, sort_keys=False, default_flow_style=False)

    print(f'spawning ranging devices in {params["world"]}')

    gazebo_command = get_gazebo_command()

    launch_description_list = []
    # spawn the lander in the rubicon world which also publishes odom message for radio navigation
    ign_service_name = '/world/' + params['world'] + '/create'
    name = params['lander_name']
    sdf_path = os.path.join(mrg_slam_sim_share_dir, 'models', params['lander_sdf_file'])
    x, y, z = params['lander_pose'][0], params['lander_pose'][1], params['lander_pose'][2]
    roll, pitch, yaw = params['lander_pose'][3], params['lander_pose'][4], params['lander_pose'][5]
    qx, qy, qz, qw = get_quaternion_from_euler(roll, pitch, yaw)

    if gazebo_command == 'ign':
        lander_process = ign_spawn_model_process(ign_service_name, sdf_path, name, x, y, z, qx, qy, qz, qw)
    elif 'gz' in gazebo_command:
        lander_process = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 
                                                       'launch', 'gz_spawn_model.launch.py')),
            launch_arguments={
                'world': params['world'],
                'file': sdf_path,
                'entity_name': 'lander',
                'x': str(x), 'y': str(y), 'z': str(z),
                'roll': str(roll), 'pitch': str(pitch), 'yaw': str(yaw)
            }.items()
        )
    print(f'spawning lander at pose x {x}, y {y}, z {z}, roll {roll}, pitch {pitch}, yaw {yaw}')
    launch_description_list.append(lander_process)

    # parse the spawn poses (x, y, z, roll, pitch, yaw) from the yaml file
    spawn_poses = np.array(params['ranging_device_poses']).reshape(-1, 6)
    # spawn a radio box for each pose
    base_name = params['ranging_device_name']
    for i, pose in enumerate(spawn_poses):
        print(f'spawning ranging device {i} at pose {pose}')
        name = base_name + str(i)

        sdf_path = os.path.join(mrg_slam_sim_share_dir, 'models', params['ranging_device_sdf_file'])
        # We create a temporary sdf file to namespace the cmd_vel and laser scan topics for use with multiple robots
        sdf_path_tmp = sdf_path.replace('.sdf', '_' + name + '.sdf')
        os.system(f'cp {sdf_path} {sdf_path_tmp}')

        namespace_sdf_file(sdf_path_tmp, name)

        x, y, z = pose[0], pose[1], pose[2]
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        qx, qy, qz, qw = get_quaternion_from_euler(roll, pitch, yaw)
        if 'ign' in gazebo_command:
            spawn_process = ign_spawn_model_process(ign_service_name, sdf_path_tmp, name, x, y, z, qx, qy, qz, qw)
        elif 'gz' in gazebo_command:
            spawn_process = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 
                                                           'launch', 'gz_spawn_model.launch.py')),
                launch_arguments={
                    'world': params['world'],
                    'file': sdf_path_tmp,
                    'entity_name': name,
                    'x': str(x), 'y': str(y), 'z': str(z),
                    'roll': str(roll), 'pitch': str(pitch), 'yaw': str(yaw)
                }.items()
            )
        launch_description_list.append(spawn_process)

    # create a ros_gz_bridge which creates a config file for the radio boxes on the fly
    ros_gz_config = os.path.join(mrg_slam_sim_share_dir, 'config', 'ranging_ros_gz_bridge.yaml')
    ros_gz_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                        '--ros-args', '-p', 'config_file:=' + ros_gz_config], output='screen')
    launch_description_list.append(ros_gz_bridge)

    return LaunchDescription(launch_description_list)
