import os
import sys
import yaml
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
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


def namespace_sdf_file(sdf_path, namespace):
    tree = ET.parse(sdf_path)
    for plugin in tree.findall('model'):
        plugin.attrib['name'] = namespace
    for plugin in tree.findall('model/plugin'):
        if plugin.attrib['name'] == 'ignition::gazebo::systems::OdometryPublisher':
            # Give the odometry publisher a unique topic name
            plugin.find('odom_topic').text = namespace + '/odom'

    tree.write(file_or_filename=sdf_path, encoding='utf-8', xml_declaration=True)


def spawn_model_process(ign_service_name, sdf_path, model_name, x, y, z, qx, qy, qz, qw):
    ign_request_args = 'sdf_filename: \"' + sdf_path + '\"' + ', name: \"' + model_name + \
        '\"' + f', pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}}}'

    cmd_string_list = ['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
                       'ignition.msgs.BooleanMsg', '--timeout', '10000', '--req', ign_request_args]

    process = ExecuteProcess(
        cmd=['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
             'ignition.msgs.Boolean', '--timeout', '10000', '--req', ign_request_args],
        output='screen')
    return process


# This launch file spawns a lander and radio boxes in the simulation world and the corresponding ros_gz_bridge
def generate_launch_description():
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')
    config_file = os.path.join(mrg_slam_sim_share_dir, 'config', 'radio_boxes_sim.yaml')
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)['radio_boxes_sim']['ros__parameters']
        yaml.dump(params, sort_keys=False, default_flow_style=False)

    print(f'spawning radio boxes in {params["world"]}')

    launch_description_list = []
    # spawn the lander in the rubicon world which also publishes odom message for radio navigation
    ign_service_name = '/world/' + params['world'] + '/create'
    name = params['lander_name']
    sdf_path = os.path.join(mrg_slam_sim_share_dir, 'models', params['lander_sdf_file'])
    x, y, z = params['lander_pose'][0], params['lander_pose'][1], params['lander_pose'][2]
    qx, qy, qz, qw = get_quaternion_from_euler(params['lander_pose'][3], params['lander_pose'][4], params['lander_pose'][5])
    lander_process = spawn_model_process(ign_service_name, sdf_path, name, x, y, z, qx, qy, qz, qw)
    print(f'spawning lander at pose {x}, {y}, {z}')
    launch_description_list.append(lander_process)

    # parse the spawn poses (x, y, z, roll, pitch, yaw) from the yaml file
    spawn_poses = np.array(params['poses']).reshape(-1, 6)
    # spawn a radio box for each pose
    base_name = params['radio_box_name']
    for i, pose in enumerate(spawn_poses):
        print(f'spawning radio box {i + 1} at pose {pose}')
        name = base_name + str(i + 1)

        sdf_path = os.path.join(mrg_slam_sim_share_dir, 'models', params['radio_box_sdf_file'])
        # We create a temporary sdf file to namespace the cmd_vel and laser scan topics for use with multiple robots
        sdf_path_tmp = sdf_path.replace('.sdf', '_' + name + '.sdf')
        os.system(f'cp {sdf_path} {sdf_path_tmp}')

        namespace_sdf_file(sdf_path_tmp, name)

        x, y, z = pose[0], pose[1], pose[2]
        qx, qy, qz, qw = get_quaternion_from_euler(pose[3], pose[4], pose[5])
        spawn_process = spawn_model_process(ign_service_name, sdf_path_tmp, name, x, y, z, qx, qy, qz, qw)
        launch_description_list.append(spawn_process)

    # create a ros_gz_bridge which creates a config file for the radio boxes on the fly
    ros_gz_config = os.path.join(mrg_slam_sim_share_dir, 'config', 'radio_boxes_ros_gz_bridge.yaml')
    ros_gz_bridge = ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                        '--ros-args', '-p', 'config_file:=' + ros_gz_config], output='screen')
    launch_description_list.append(ros_gz_bridge)

    return LaunchDescription(launch_description_list)
