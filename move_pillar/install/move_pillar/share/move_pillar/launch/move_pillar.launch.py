import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'move_pillar'

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf/turtlebot3_burger.urdf')
    rviz_config = os.path.join(get_package_share_directory(PACKAGE_NAME), 'rviz', 'move_pillar.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        ),
        Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        ),
        Node(
            package=PACKAGE_NAME,
            executable='move_pillar',
            name='move_pillar',
            output='screen'
        )
    ])
