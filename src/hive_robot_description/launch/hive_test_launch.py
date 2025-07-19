from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_sim.actions import GzServer  # Action pour lancer Gazebo

def generate_launch_description():
    # Déclarer l'argument 'world_sdf_file' pour le chemin du fichier SDF du monde
    declare_world_sdf_file_cmd = DeclareLaunchArgument(
        'world_sdf_file', 
        default_value='/home/thomas24/hiveverse_ws/src/hive_robot_description/worlds/hive_test.world.sdf',  # Chemin absolu
        description='Path to the SDF world file'
    )

    # Création de la description de lancement
    ld = LaunchDescription([
        # Ajouter l'action pour déclarer l'argument
        declare_world_sdf_file_cmd,

        # Lancer le serveur Gazebo avec le fichier SDF spécifié
        GzServer(
            world_sdf_file=LaunchConfiguration('world_sdf_file')  # Utiliser l'argument du fichier SDF
        ),
    ])

    return ld
