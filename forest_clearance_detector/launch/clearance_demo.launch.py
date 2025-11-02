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
            # parameters=[{
            #     #stuff that might be changed
            #     'pointcloud_topic': '/camera/depth/points',
            #     'camera_frame': 'camera_link',
            #     'base_frame': 'base_link',
            #     'target_frame': 'map',
            #     'cmd_vel_topic': '/cmd_vel',
            #     #scanning behaviour:
            #     'sector_width_rad': 2.094,   # 120 deg as specified by RGBD camera FOV (Field of View)
            #     'sectors': 3,
            #     # goal specification:
            #     'max_goals_per_sector': 2,
            #     'min_clear_columns': 6,
            #     'free_range_m': 3.0,
            #     'rotate_speed': 0.8,
            # }]
        )
    ])