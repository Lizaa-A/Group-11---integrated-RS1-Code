from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ugv_dashboard_gui',
            executable='gui_dashboard',
            name='ugv_dashboard_gui',
            output='screen'
        )
    ])
