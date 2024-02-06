import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mrg_slam_sim_share_dir = get_package_share_directory('mrg_slam_sim')
    ros_gz_sim_share_dir = get_package_share_directory('ros_ign_gazebo')

    gz_args = os.path.join(mrg_slam_sim_share_dir, 'worlds', 'rubicon.sdf')
    gz_args += " -v 4"
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': gz_args ,'ign_version': '6'}.items(),
    )

    return LaunchDescription([gz_sim])
