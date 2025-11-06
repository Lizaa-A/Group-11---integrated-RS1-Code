# This will be a launch file that gives options to use SLAM to save a map,
# or AMCL to create localisation

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution) # allows for boolean logic "auto_map" and "nav2"
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    # Get paths to directories
    pkg_path = FindPackageShare('41068_ignition_bringup')
    config_path = PathJoinSubstitution([pkg_path,
                                       'config'])

    # Additional command line arguments
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Flag to enable use_sim_time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(use_sim_time_launch_arg)

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Flag to launch RViz'
    )
    ld.add_action(rviz_launch_arg)

    nav2_launch_arg = DeclareLaunchArgument(
        'nav2',
        default_value='True',
        description='Flag to launch Nav2'
    )
    ld.add_action(nav2_launch_arg)

    #choosing mapping or localisation: "auto_map:True, means SLAM on"
    auto_map_arg = DeclareLaunchArgument(
        'auto_map', default_value='True',
        description='True: run SLAM (mapping); False: run AMCL with a static map'
    )
    ld.add_action(auto_map_arg)
    auto_map = LaunchConfiguration('auto_map')

    # path to map yaml, will be used for localisation:
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([config_path, 'husky_map.yaml']),
        description='Path to saved map YAML (used when auto_map:=False)'
    )
    ld.add_action(map_yaml_arg)
    map_yaml = LaunchConfiguration('map_yaml')

    # mirroring current config into bringup_launch.py file - for consistency
    params_file_arg = DeclareLaunchArgument(
    'params_file',
    default_value=PathJoinSubstitution([config_path, 'nav2_params.yaml']),
    description='Nav2 parameters file'
    )
    ld.add_action(params_file_arg)
    params_file = LaunchConfiguration('params_file')

    # Load robot_description and start robot_state_publisher
    robot_description_content = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([pkg_path,
                                       'urdf',
                                       'husky.urdf.xacro'])]),
        value_type=str)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time': use_sim_time
                                      }])
    ld.add_action(robot_state_publisher_node)

    # Publish odom -> base_link transform **using robot_localization**
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_localization',
        output='screen',
        parameters=[PathJoinSubstitution([config_path,
                                          'robot_localization.yaml']),
                    {'use_sim_time': use_sim_time}]
    )
    ld.add_action(robot_localization_node)

    # Start Gazebo to simulate the robot in the chosen world
    world_launch_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_trees',
        description='Which world to load',
        choices=['simple_trees', 'large_demo', 'flat_terrain_rocks', 'flat_terrain_trees', 'my_custom_demo']
    )
    ld.add_action(world_launch_arg)
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                             'launch', 'ign_gazebo.launch.py']),
        launch_arguments={
            'ign_args': [PathJoinSubstitution([pkg_path,
                                               'worlds',
                                               [LaunchConfiguration('world'), '.sdf']]),
                         ' -r']}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    robot_spawner = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-topic', '/robot_description', '-z', '0.4']
    )
    ld.add_action(robot_spawner)

    # Bridge topics between gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': PathJoinSubstitution([config_path,
                                                          'gazebo_bridge.yaml']),
                    'use_sim_time': use_sim_time}]
    )
    ld.add_action(gazebo_bridge)

    # rviz2 visualises data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([config_path,
                                               '41068.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    ld.add_action(rviz_node)

    # Nav2 enables mapping and waypoint following
    nav_auto_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_path, 'launch', '41068_navigation.launch.py'])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        # Condition: run this include only if (auto_map == True) AND (nav2 == True)
        condition=IfCondition(LaunchConfiguration('auto_map'))
    )
    ld.add_action(nav_auto_map)

    map_saver_server = Node(
        package='nav2_map_server', executable='map_saver_server',
        name='map_saver_server', output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('auto_map'))
    )
    ld.add_action(map_saver_server)

    # Autostart the lifecycle node "map_saver_server" so /map_saver_server/save_map is available
    lifecycle_mgr_map_saver = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map_saver', output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_saver_server']   # must match the 'name' of your saver node above
        }],
        condition=IfCondition(LaunchConfiguration('auto_map'))
    )
    ld.add_action(lifecycle_mgr_map_saver)

    nav_localise = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,   # use your YAML for costmaps/planner/controller
            'map': map_yaml              # static map path enables map_server + AMCL
        }.items(),
        # Condition: run this include only if (auto_map == False) AND (nav2 == True)
        condition=UnlessCondition(LaunchConfiguration('auto_map'))
    )
    ld.add_action(nav_localise)


    return ld