#!/usr/bin/env python3
"""
Full Pipeline:
1. rotating_clearance_detector → /ugv/clearance_goals_unfiltered
2. costmap_goal_filter         → /ugv/clearance_goals_unordered + /ugv/clearance_goals_rejected
3. goals_markers               → RED circles on REJECTED goals only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # CHANGE THIS TO YOUR ACTUAL PACKAGE NAME
    pkg_name = 'forest_clearance_detector'

    return LaunchDescription([

        # === 1. Rotating Clearance Detector ===
        Node(
            package=pkg_name,
            executable='rotating_clearance_detector',
            name='rotating_clearance_detector',
            output='screen',
            parameters=[{
                # 'pointcloud_topic': '/camera/depth/points',
                # 'camera_frame': 'camera_link',
                # 'base_frame': 'base_link',
                # 'target_frame': 'map',
                # 'cmd_vel_topic': '/cmd_vel',
                # 'sector_width_rad': 1.047,      # 60°
                # 'sectors': 6,
                # 'max_goals_per_sector': 2,
                # 'min_clear_columns': 10,
                # 'num_columns': 150,
                # 'max_xy': 4.0,
                # 'obstacle_z_thresh': 0.02,
                # 'free_range_m': 2.0,
                # 'rotate_speed': 0.8,
                # 'rotate_settle_s': 1.2,
            }]
        ),

        # === 2. Costmap Goal Filter ===
        Node(
            package=pkg_name,
            executable='costmap_goal_filter',
            name='costmap_goal_filter',
            output='screen',
            parameters=[{
                # 'input_topic': '/ugv/clearance_goals_unfiltered',
                # 'output_topic': '/ugv/clearance_goals_unordered',
                # 'global_costmap_topic': 'global_costmap/costmap',
                # 'global_frame': 'map',
                # 'lethal_threshold': 100,
                # 'inflated_threshold': 99,
                # 'filter_inflated': True,
            }]
        ),

        # === 3. REJECTED GOALS → RED CIRCLES ONLY ===
        Node(
            package=pkg_name,
            executable='goals_markers',
            name='rejected_goals_markers',
            output='screen',
            parameters=[{
                # 'in_topic': '/ugv/clearance_goals_rejected',
                # 'marker_topic': '/mission/markers/rejected',
                # 'show_text': True,
                # 'z_offset': 0.15,
                # 'scale': 0.35,
                # 'text_scale': 0.3,
                # 'color_r': 1.0,
                # 'color_g': 0.0,
                # 'color_b': 0.0,
                # 'color_a': 1.0,
            }]
        ),
    ])