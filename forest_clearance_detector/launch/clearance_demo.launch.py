# clearance_demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forest_clearance_detector',
            executable='rotating_clearance_detector',
            name='rotating_clearance_detector',
            output='screen',
        )
    ])