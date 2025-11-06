# BELOW is FULL LAUNCH WITH SIM + FSM + GUI + PERCEPTION + HUSKY_NAV + CONDITIONS + PLANTING

# Need to better sync up statuses - especially with the conditon part
'''
ros2 launch mission_fsm mission_fsm_gui_ignition.launch.py \
  rviz:=true \
  nav2:=true \
  use_sim_time:=true \
  auto_map:=false \
  map_yaml:=$(ros2 pkg prefix mission_fsm)/share/mission_fsm/maps/siteA.yaml

ros2 launch mission_fsm mission_fsm_gui_ignition.launch.py \
  rviz:=true \
  nav2:=true  \
  use_sim_time:=true  \
  auto_map:=false  \
  world:=flat_terrain_rocks  \
  map_yaml:=$(ros2 pkg prefix mission_fsm)/share/mission_fsm/maps/FlatTerrainRockssiteA.yaml

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
    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    auto_map = LaunchConfiguration('auto_map')
    map_yaml = LaunchConfiguration('map_yaml')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2')
    declare_nav2 = DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2 stack')
    declare_auto_map = DeclareLaunchArgument('auto_map', default_value='false', description='If true, run mapping (slam) instead of AMCL')
    declare_map_yaml = DeclareLaunchArgument('map_yaml', default_value='', description='Path to a map .yaml file for AMCL')

    # -------------------------------------------------
    # include my existing sim / ignition bringup
    # ros2 launch 41068_ignition_bringup 41068_custom.launch.py ...
    # -------------------------------------------------
    ignition_pkg = FindPackageShare('41068_ignition_bringup')

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ignition_pkg,
                'launch',
                '41068_custom.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'nav2': nav2,
            'auto_map': auto_map,
            'map_yaml': map_yaml,
        }.items()
    )

    # -------------------------------------------------
    # mission FSM nod
    # -------------------------------------------------
    fsm_node = Node(
        package='mission_fsm',
        executable='mission_fsm_node',
        name='mission_fsm',
        output='screen',
    )

    # -------------------------------------------------
    # PySide6 dashboard GUI
    # -------------------------------------------------
    gui_node = Node(
        package='ugv_dashboard_gui',
        executable='gui_dashboard',
        name='ugv_dashboard_gui',
        output='screen'
    )


    # -------------------------------------------------
    # build launch description
    # -------------------------------------------------
    ld = LaunchDescription()

    # args
    ld.add_action(declare_use_sim_time)
    # ld.add_action(telemetry_node)
    ld.add_action(declare_rviz)
    ld.add_action(declare_nav2)
    ld.add_action(declare_auto_map)
    ld.add_action(declare_map_yaml)

    # included sim + my nodes
    ld.add_action(include_sim)
    ld.add_action(fsm_node)
    ld.add_action(gui_node)

    return ld