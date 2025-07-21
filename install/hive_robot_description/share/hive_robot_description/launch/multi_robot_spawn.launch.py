################################################################################
# Launch file to spawn two robots in Gazebo under different namespaces/positions,
# with a delay for the second robot to avoid Ogre2/Gazebo material bug.
################################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
import xacro

def make_robot_group(namespace, x_pose, y_pose, z_pose=0.0, yaw=0.0):
    pkg = 'hive_robot_description'
    model_file = os.path.join(
        get_package_share_directory(pkg), 'model', 'mk5_200.urdf.xacro'
    )
    # Le PATCH est ICI : on passe robot_ns à xacro
    robot_desc = xacro.process_file(model_file, mappings={'robot_ns': namespace}).toxml()

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', namespace,
            '-topic', 'robot_description',
            '-x', str(x_pose),
            '-y', str(y_pose),
            '-z', str(z_pose),
            '-Y', str(yaw)
        ],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': f'{namespace}/'
        }]
    )

    bridge_params = os.path.join(
        get_package_share_directory(pkg), 'parameters', 'bridge_parameters.yaml'
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    return GroupAction([
        PushRosNamespace(namespace),
        spawn,
        rsp,
        bridge
    ])

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ['-r -v 4 empty.sdf'], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot1 immédiatement
    robot1 = make_robot_group('robot1', x_pose=0, y_pose=0)

    # Spawn robot2 après 10 secondes (tu peux réduire à 2.0 quand tout marche)
    robot2 = TimerAction(
        period=10.0,  # délai en secondes
        actions=[make_robot_group('robot2', x_pose=2, y_pose=0)]
    )

    return LaunchDescription([
        gazebo_launch,
        robot1,
        robot2,
    ])
