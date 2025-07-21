import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import xacro

def launch_setup(context, *args, **kwargs):
    ns_value = LaunchConfiguration("namespace").perform(context)

    pkg_name       = 'hive_robot_description'
    xacro_rel_path = 'model/mk5_200.urdf.xacro'
    xacro_abs_path = os.path.join(
        get_package_share_directory(pkg_name), xacro_rel_path
    )

    # IMPORTANT : passer le namespace au xacro !
    robot_description = xacro.process_file(
        xacro_abs_path,
        mappings={'robot_ns': ns_value}
    ).toxml()

    # Spawn model in Gazebo (déjà en cours)
    spawn_node = Node(
        package    = 'ros_gz_sim',
        executable = 'create',
        arguments  = ['-name', ns_value, '-topic', 'robot_description'],
        output     = 'screen',
    )

    # Robot State Publisher (avec frame_prefix)
    rsp_node = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{
            'robot_description': robot_description,
            'use_sim_time'    : True,
            'frame_prefix'    : f"{ns_value}/"     # robot1/odom, robot2/base_link, …
        }],
    )

    bridge_params = os.path.join(
        get_package_share_directory(pkg_name),
        'parameters',
        'bridge_parameters.yaml'
    )
    bridge_node = Node(
        package    = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments  = ['--ros-args', '-p', f'config_file:={bridge_params}'],
        output     = 'screen',
    )

    robot_group = GroupAction(
        actions = [
            PushRosNamespace(ns_value),
            spawn_node,
            rsp_node,
            bridge_node,
        ]
    )

    return [robot_group]

def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="robot1",
        description="Namespace for this robot instance"
    )

    return LaunchDescription([
        ns_arg,
        OpaqueFunction(function=launch_setup)
    ])
