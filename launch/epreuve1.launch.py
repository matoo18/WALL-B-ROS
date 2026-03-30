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

    suivi_traj = Node(
        package='projet',
        executable='suivi',
        name='suivi',
        output='screen',
        on_exit=Shutdown(),
        parameters=[
            {'roundabout_dir': 'right'}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(environnement)
    ld.add_action(suivi_traj)

    return ld