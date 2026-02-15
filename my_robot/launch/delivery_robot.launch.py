from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',                    # nom de ton package
            executable='move_to_zone.py',          # ton script python
            name='delivery_robot',                 # nom du noeud
            output='screen',                       # afficher les logs dans le terminal
        )
    ])
