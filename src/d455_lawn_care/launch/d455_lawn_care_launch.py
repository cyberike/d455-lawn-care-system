#!/usr/bin/env python3
"""
D455 Lawn Care Launch File with Tuned Parameters
Launches D455 camera and simplified lawn care detection nodes
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('d455_lawn_care')
    
    # Configuration file path
    tuned_params_file = os.path.join(pkg_dir, 'config', 'tuned_detection_params.yaml')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # RealSense D455 Camera Node
    d455_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455_camera',
        namespace='d455',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'device_type': 'd455',
                'enable_sync': True,
                'align_depth': True,
                'enable_pointcloud': False,  # Disabled for performance
                
                # Stream configurations optimized for lawn care
                'enable_color': True,
                'color_width': 848,    # Reduced for better performance
                'color_height': 480,
                'color_fps': 30,
                
                'enable_depth': True,
                'depth_width': 848,
                'depth_height': 480,
                'depth_fps': 30,
                
                'enable_infra1': True,
                'enable_infra2': True,
                'infra_width': 848,
                'infra_height': 480,
                'infra_fps': 30,
                
                # Outdoor optimizations
                'laser_power': 360,     # Maximum laser power for outdoors
                'exposure': 8000,       # Fixed exposure
                'gain': 16,             # Low noise gain
                
                # Disable unnecessary streams
                'enable_fisheye': False,
                'enable_gyro': False,
                'enable_accel': False,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        emulate_tty=True,
    )
    
    # Simple Grass Detection Node with tuned parameters
    grass_detector_node = Node(
        package='d455_lawn_care',
        executable='grass_detector_simple',
        name='simple_grass_detector',
        parameters=[
            tuned_params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )
    
    # Simple Obstacle Detection Node with tuned parameters
    obstacle_detector_node = Node(
        package='d455_lawn_care',
        executable='obstacle_detector_simple',
        name='simple_obstacle_detector',
        parameters=[
            tuned_params_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )
    
    # Static Transform Publishers
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_d455_tf',
        arguments=[
            '0.2', '0', '0.3',           # x, y, z (20cm forward, 30cm up)
            '0', '0.17', '0',            # roll, pitch, yaw (10° down tilt)
            'base_link',
            'd455_link'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d455_optical_tf',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',  # Optical frame quaternion
            'd455_link',
            'd455_color_optical_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_log_level)
    
    # Add informational message
    ld.add_action(LogInfo(msg="Starting D455 Lawn Care System with Tuned Parameters"))
    
    # Add all nodes
    ld.add_action(d455_camera_node)
    ld.add_action(grass_detector_node)
    ld.add_action(obstacle_detector_node)
    ld.add_action(base_to_camera_tf)
    ld.add_action(camera_optical_tf)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()