import os
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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


COMMAND_LINE_ARGUMENTS = [
    'robot_name',
    'world',
    'sdf_file',
    'x',
    'y',
    'z',
    'roll',
    'pitch',
    'yaw'
]


def generate_launch_description():

    for arg in sys.argv:
        print(arg)

    mrg_sim_share_dir = get_package_share_directory('mrg_sim')
    print('asdjkfhaskljdhalskjdfh')
    # Declare robot name argument
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='husky',
        description='Specifies the robot name to be used in ROS topics.')
    robot_name = LaunchConfiguration('robot_name').perform(None)

    print(robot_name)
    # Declare world argument
    world = DeclareLaunchArgument(
        'world',
        default_value='challenge',
        description='Specifies the world to be used in Gazebo.')
    # Declare sdf file argument
    sdf = DeclareLaunchArgument(
        'sdf_file',
        default_value='COSTAR_HUSKY_SENSOR_CONIG_1/model.sdf',
        description='Specifies the sdf file to be used in Gazebo.')
    # Declare robot starting position for each robot argument x y z
    x = DeclareLaunchArgument(
        'x',
        default_value=0.0,
        description='Specifies the robot starting position in Gazebo.')
    y = DeclareLaunchArgument(
        'y',
        default_value=0.0,
        description='Specifies the robot starting position in Gazebo.')
    z = DeclareLaunchArgument(
        'z',
        default_value=0.0,
        description='Specifies the robot starting position in Gazebo.')
    # Declare robot starting orientation argument roll pitch yaw
    roll = DeclareLaunchArgument(
        'roll',
        default_value=0.0,
        description='Specifies the robot starting orientation in Gazebo.')
    pitch = DeclareLaunchArgument(
        'pitch',
        default_value=0.0,
        description='Specifies the robot starting orientation in Gazebo.')
    yaw = DeclareLaunchArgument(
        'yaw',
        default_value=0.0,
        description='Specifies the robot starting orientation in Gazebo.').value

    sdf_path = os.path.join(mrg_sim_share_dir, 'models', sdf)
    ign_service_name = '/world/ + ' + world + '/create'

    qx, qy, qz, qw = get_quaternion_from_euler(roll, float(pitch), float(yaw))

    ign_request_args = 'sdf_filename \"' + sdf_path + '\"' + ', name \"' + robot_name + \
        f', pose: {{ position: {{ x: {x}, y: {y}, z: {z} }}, orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}}}'

    return LaunchDescription([
        robot_name,
        world,
        sdf,
        x,
        y,
        z,
        roll,
        pitch,
        yaw,
        ExecuteProcess(
            cmd=['ign', 'service', '-s', ign_service_name, '--reqtype', 'ignition.msgs.EntityFactory', '--reptype',
                 'ignition.msgs.BooleanMsg', '--timeout', '10000', '--req', ign_request_args],
            output='screen', arguments=[ign_request_args])])
