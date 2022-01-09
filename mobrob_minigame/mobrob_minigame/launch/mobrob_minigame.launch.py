from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_NAME = 'mobrob_minigame'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package=PACKAGE_NAME,
            executable='mobrob_minigame',
            name='mobrob_minigame',
            output='screen'
        ),
        Node(
            package='zad33',
            executable='seekAndDestroy',
            name='seekAndDestroy'
        )
    ])
