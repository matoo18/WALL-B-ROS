import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_projet2025 = get_package_share_directory('projet2025')

    environnement = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_projet2025, 'launch', 'projet.launch.py')
        )
    )

    vision = Node(
        package='projet',
        executable='omniscience',
        name='maestro',
        output='screen',
        on_exit=Shutdown()
    )

    maestro = Node(
        package='projet',
        executable='maestro',
        name='maestro',
        output='screen',
        on_exit=Shutdown()
    )

    epreuve1_suivi = Node(
        package='projet',
        executable='suivi',
        name='suivi',
        output='screen',
        on_exit=Shutdown(),
        parameters=[
            {'roundabout_dir': 'right'}
        ]
    )

    epreuve2_obstacle = Node(
        package='projet',
        executable='obstacle',
        name='obstacle',
        output='screen',
        on_exit=Shutdown()
    )


    ld = LaunchDescription()

    ld.add_action(environnement)
    
    ld.add_action(vision)
    ld.add_action(maestro)

    ld.add_action(epreuve1_suivi)
    ld.add_action(epreuve2_obstacle)

    return ld