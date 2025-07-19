from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    pkg = get_package_share_directory('hive_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'mk5_200.urdf.xacro')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[robot_description]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             output='screen')
    ])
