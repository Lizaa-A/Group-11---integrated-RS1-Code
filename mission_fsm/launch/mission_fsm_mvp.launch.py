from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_fsm',
            executable='mission_fsm_node',
            name='mission_fsm',
            output='screen',
            parameters=[{
                # add params here later if needed
            }]
        )
    ])
