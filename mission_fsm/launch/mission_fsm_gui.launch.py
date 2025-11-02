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
        ),
        # your PySide6 GUI
        Node(
            package='ugv_dashboard_gui',
            executable='gui_dashboard',
            name='ugv_dashboard_gui',
            output='screen'
        ),

        # ignition node w environment and mapping
        Node(
            package='ignition_ros2',
            executable='ignition_node',
            name='ignition_simulation',
            output='screen',
            arguments=['-r', 'path/to/your/world.sdf']
        ),
    ])

'''
ros2 launch mission_fsm mission_fsm_gui.launch.py
'''
# ABOVE is ONLY FSM + GUI