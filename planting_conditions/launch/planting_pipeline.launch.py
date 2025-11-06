# planting_pipeline.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    declare_cmd_vel = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')

    # 0) Start the single-port TCP→ROS2 sensor bridge FIRST
    sensor_bridge = Node(
        package='planting_conditions',
        executable='tcp_to_ros2_both.py',  
        name='tcp_server_to_ros2_sensors',
        output='screen',
        respawn=True,
        parameters=[{
            'listen_host': '0.0.0.0',
            'port': 5005,              
            'sun_raw_threshold': 15,
            'dry_adc': 150,
            'wet_adc': 400,
        }],
    )
    
    seed_weight = Node(
        package='planting_conditions',
        executable='seed_weight',
        name='planting_weight',
        output='screen',
        parameters=[{
            'raw_zero': 0.0,
            'raw_known': 10000.0,
            'known_weight_g': 1000.0,
            'empty_holder_g': 120.0,
            'seed_mass_g': 0.040,
            'low_threshold': 50,
            'smooth_window': 10,
            'publish_step_g': 5.0,
            'raw_msg_type': 'float32',
        }],
    )

    planting_site_markers = Node(
        package='planting_conditions',
        executable='planting_site_markers',   # C++ node we added
        name='planting_site_markers',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'base_frame': 'base_link',
            'marker_topic': '/mission/site_markers',
            'drop_on_sun_ok': True,              
            'reject_reason_debounce_ms': 150,     
            'sun_ok_max_age_s': 1.0,              
            'x_size_m': 0.6,
            'square_size_m': 0.35,
            'circle_diam_m': 0.5,
            'marker_z': 0.05,
            'line_width_m': 0.05,
        }],
    )

    # 1)tank simulator always on
    water_level_sim = Node(
        package='planting_conditions',
        executable='water_level_sim',
        name='water_level_sim',
        output='screen',
        parameters=[{
            'tank_volume_l': 10.0,
            'initial_level_percent': 100.0,
            'leak_lps': 0.0,
            'low_threshold_percent': 15.0,
            'stop_when_empty': True,
            'publish_step_percent': 5.0,
        }],
    )

    # 2) task nodes (idle until orchestrator triggers services)
    soil_prep = Node(
        package='planting_conditions',
        executable='soil_prep_fsm',
        name='soil_prep_fsm',
        output='screen',
        parameters=[{
            'raw_accept_thresh': 400,
            'moisture_topic': '/soil/moisture_raw',
            'rake_speed_mps': 0.2,
            'rake_passes': 3,
            'rake_segment_time_s': 2.0,
            'cmd_vel_topic': cmd_vel_topic,
            'sunlight_topic': '/sunlight/ok',     
            'sunlight_required': True,
            'sunlight_max_age_s': 3.0,
        }],
    )
    
    # delayed_soil_prep = TimerAction(period=0.5, actions=[soil_prep])

    planting = Node(
        package='planting_conditions',
        executable='planting_sim',
        name='planting_sim',
        output='screen',
        parameters=[{
            'plant_duration_s': 5.0,
            'plant_speed_mps': 0.15,
            'seeds_per_run': 10,
            'cmd_vel_topic': cmd_vel_topic,
        }],
    )

    cover = Node(
        package='planting_conditions',
        executable='cover_sim',
        name='cover_sim_no_odom',
        output='screen',
        parameters=[{
            'back_speed_mps': 0.10,
            'cover_fallback_time_s': 3.0,
            'passes': 1,
            'cmd_vel_topic': cmd_vel_topic,
        }],
    )

    irrigation = Node(
        package='planting_conditions',
        executable='irrigation_sim',
        name='irrigation_sim',
        output='screen',
        parameters=[{
            'water_speed_mps': 0.12,
            'flow_lps_cmd': 1,
            'stop_on_low': True,
            'auto_refill_on_low': False,
            'fallback_time_s': 6.0,
            'cmd_vel_topic': cmd_vel_topic,
            'odom_topic': '/odom',
        }],
    )

    # 3) Orchestrator (DELAYED) — give the TCP bridge time to connect and start publishing
    orchestrator = Node(
        package='planting_conditions',
        executable='planting_pipeline.py',  #
        name='planting_pipeline',
        output='screen',
    )
    delayed_orchestrator = TimerAction(period=3.0, actions=[orchestrator])

    return LaunchDescription([
        declare_cmd_vel,
        sensor_bridge,         # start first
        planting_site_markers,
        water_level_sim,
        seed_weight,
        soil_prep,
        planting,
        cover,
        irrigation,
        delayed_orchestrator,  # start last, after a short delay
    ])