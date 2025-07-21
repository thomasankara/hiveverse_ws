import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Remplace par le chemin de ton xacro/urdf
    pkg = 'hive_robot_description'
    model_file = os.path.join(
        get_package_share_directory(pkg), 'model', 'mk5_200.urdf.xacro'
    )

    # Génére le robot_description depuis le xacro (optionnel : tu peux aussi passer directement le urdf)
    robot_desc = xacro.process_file(model_file, mappings={'robot_ns': 'robot1'}).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',           # --> Ajoute le namespace ici
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
            {'frame_prefix': 'robot1/'} # --> Pour avoir des frames "robot1/base_link", etc.
        ],
        output='screen'
    )

    return LaunchDescription([rsp])
