from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg = get_package_share_directory('hive_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'mk5_200.urdf.xacro')
    # Génère un URDF, puis gz le convertira à la volée (possible)
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', '-r',  # -r=run
                 # Monde vide par défaut
                 ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['bash','-c',
                 f'xacro {xacro_file} > /tmp/mk5_200.urdf && '
                 f'gz sim -v4 -g & sleep 3 && '
                 f'ros2 run ros_gz_sim create -file /tmp/mk5_200.urdf -name mk5_200'],
            shell=False
        )
    ])
