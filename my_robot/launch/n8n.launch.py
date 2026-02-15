
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',                    # nom de ton package
            executable='ros_web_bridge.py',          # ton script python
            name='ros_web_bridge',                 # nom du noeud
            output='screen',       
            emulate_tty=True,
            parameters=[]  ,                     # afficher les logs dans le terminal
        )
    ])
