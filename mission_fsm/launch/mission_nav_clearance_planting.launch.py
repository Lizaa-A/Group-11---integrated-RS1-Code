# BELOW is FULL LAUNCH WITH SIM + FSM + GUI + PERCEPTION + HUSKY_NAV + CONDITIONS + PLANTING
# Need to better sync up statuses - especially with the conditon part
'''
ros2 launch mission_fsm mission_nav_clearance_planting.launch.py


'''
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------
    # launch arguments (same as in my 41068_custom.launch.py)
    # -------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    initpose = LaunchConfiguration('publish_initialpose')
    require_perm = LaunchConfiguration('require_permission')


    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    declare_initpose = DeclareLaunchArgument('publish_initialpose', default_value='true')
    declare_permission = DeclareLaunchArgument('require_permission', default_value='true', description='If true, goal_tracker will wait for permission to send each goal except first goal')

    # -------------------------------------------------
    # mission nav node
    # -------------------------------------------------
    husky_nav_goal_share = FindPackageShare('husky_nav_goal')

    include_mission_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                husky_nav_goal_share,
                'launch',
                'mission_nav.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_initialpose': initpose,
            'require_permission': require_perm,
        }.items(),
    )

    # -------------------------------------------------
    # planting and conditions node
    # -------------------------------------------------
    planting_conditions_share = FindPackageShare('planting_conditions')

    include_planting_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                planting_conditions_share,
                'launch',
                'planting_pipeline.launch.py'
            ])
        ),
        launch_arguments={
        #     'use_sim_time': use_sim_time,
        #     'publish_initialpose': initpose,
        #     'require_permission': require_perm,
        }.items(),
    )

    # -------------------------------------------------
    # forest clearance detector node, might have to change this later to be pkg if adding costmap stuff into forest_clearance_detector
    # -------------------------------------------------
    clearance_node = Node(
            package='forest_clearance_detector',
            executable='rotating_clearance_detector',
            name='rotating_clearance_detector',
            output='screen'
    )

    # -------------------------------------------------
    # build launch description
    # -------------------------------------------------
    ld = LaunchDescription()

    # args
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_initpose)
    ld.add_action(declare_permission)

    ld.add_action(clearance_node)
    ld.add_action(include_mission_nav)
    ld.add_action(include_planting_pipeline)

    return ld