import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_NAME = 'trackme'


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory(PACKAGE_NAME), 'rviz', 'trackme.rviz')

    return LaunchDescription([
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        ),
        Node(
            package=PACKAGE_NAME,
            executable='trackme',
            name='trackme',
            output='screen'
        )
    ])
