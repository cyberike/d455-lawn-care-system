#!/usr/bin/env python3
"""
Launch file for Laser Weed Incisor system
Integrates with existing D455 lawn care system for precision weed control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'laser_enabled',
            default_value='false',
            description='Enable laser firing capabilities (SAFETY CRITICAL)'
        ),
        
        DeclareLaunchArgument(
            'auto_targeting',
            default_value='false', 
            description='Enable automatic weed targeting based on grass detection'
        ),
        
        DeclareLaunchArgument(
            'laser_power_percentage',
            default_value='80.0',
            description='Laser power as percentage of maximum (0-100)'
        ),
        
        DeclareLaunchArgument(
            'safety_confirmation_required',
            default_value='true',
            description='Require explicit safety confirmation before firing'
        ),
        
        DeclareLaunchArgument(
            'debug_mode',
            default_value='true',
            description='Enable debug visualization and extra logging'
        ),
        
        # Safety warning
        LogInfo(msg=[
            '='*80,
            '\n*** LASER SAFETY WARNING ***',
            '\nClass 4 laser system - Invisible laser radiation',
            '\nAvoid eye or skin exposure to direct or scattered radiation',
            '\nEnsure all personnel wear certified laser safety glasses',
            '\nVerify safety zone is clear before enabling laser',
            '\n', '='*80
        ]),
        
        # Laser Weed Incisor Node
        Node(
            package='d455_lawn_care',
            executable='laser_weed_incisor',
            name='laser_weed_incisor',
            parameters=[
                # Laser control parameters
                {'laser_power_percentage': LaunchConfiguration('laser_power_percentage')},
                {'exposure_time_ms': 200},
                {'targeting_precision_mm': 1.0},
                {'auto_fire_enabled': LaunchConfiguration('auto_targeting')},
                
                # Safety parameters  
                {'safety_confirmation_required': LaunchConfiguration('safety_confirmation_required')},
                {'emergency_stop_enabled': True},
                {'max_fire_duration_ms': 500},
                {'cooldown_period_ms': 1000},
                {'safety_zone_radius_m': 2.0},
                
                # Detection integration
                {'min_weed_confidence': 0.7},
                {'max_weed_size_pixels': 5000},
                {'targeting_frame_id': 'd455_color_optical_frame'},
                
                # Debug and logging
                {'enable_debug_visualization': LaunchConfiguration('debug_mode')},
                {'log_targeting_stats': LaunchConfiguration('debug_mode')},
            ],
            remappings=[
                ('~/grass_detection', '/grass_detector/grass_detection'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('laser_enabled'))
        ),
        
        # Safety Monitor Node (always runs when laser node is active)
        Node(
            package='d455_lawn_care',
            executable='laser_safety_monitor',
            name='laser_safety_monitor', 
            parameters=[
                {'emergency_stop_enabled': True},
                {'proximity_sensor_enabled': False},  # Set true if hardware available
                {'operator_distance_min_m': 5.0},
                {'safety_zone_monitoring': True},
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('laser_enabled'))
        ),
        
        # Laser Control GUI (optional)
        Node(
            package='d455_lawn_care',
            executable='laser_control_gui',
            name='laser_control_gui',
            parameters=[
                {'show_targeting_overlay': LaunchConfiguration('debug_mode')},
                {'enable_manual_targeting': True},
                {'safety_confirmation_gui': LaunchConfiguration('safety_confirmation_required')},
            ],
            condition=IfCondition(LaunchConfiguration('debug_mode'))
        ),
        
    ])