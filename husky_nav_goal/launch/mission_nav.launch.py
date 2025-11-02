from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim_time')
    initpose = LaunchConfiguration('publish_initialpose')
    require_perm = LaunchConfiguration('require_permission')

    return LaunchDescription([
        # --- args ---
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_initialpose', default_value='true'),
        # this is so the future BT package can flip it to true when it includes this launch
        DeclareLaunchArgument('require_permission', default_value='true'),

        # --- goals_markers ---
        Node(
            package='husky_nav_goal',
            executable='goals_markers',
            output='screen',
            parameters=[{
                'in_topic': '/mission/waypoints',
                'use_sim_time': use_sim,
            }]
        ),

        # --- tsp_clearance_goals (unordered -> ordered) ---
        Node(
            package='husky_nav_goal',
            executable='tsp_clearance_goals',
            output='screen',
            parameters=[{
                'in_topic': 'ugv/clearance_goals_unordered',
                'out_topic': '/mission/waypoints',
                'map_frame': 'map',
                'force_map_frame': True,
                'seed_from_amcl': True,
                'use_sim_time': use_sim,
            }]
        ),

        # --- goal_nav (single-goal client to /navigate_to_pose) ---
        Node(
            package='husky_nav_goal',
            executable='goal_nav',
            output='screen',
            parameters=[{
                'expected_frame': 'map',
                'cancel_if_new_goal': True,
                'use_sim_time': use_sim,
            }]
        ),

        # --- goal_tracker (queue manager) ---
        Node(
            package='husky_nav_goal',
            executable='goal_tracker',
            output='screen',
            parameters=[{
                'expected_frame': 'map',
                # today: no BT , let it auto-advance
                # future BT: include this launch and pass require_permission:=true
                'require_permission': require_perm,
                'use_sim_time': use_sim,
            }]
        ),

        # --- one-shot initialpose ---
        TimerAction(
            period=15.0,   # seconds
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-lc',
                        # single-quoted YAML exactly like terminal command
"""ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '
header:
  frame_id: map
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {z: 0.0, w: 1.0}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.01, 0, 0, 0,
               0, 0, 0, 0.01, 0, 0,
               0, 0, 0, 0, 0.01, 0,
               0, 0, 0, 0, 0, 0.0685]'
"""
                    ],
                    condition=IfCondition(initpose)
                )
            ]
        ),
    ])