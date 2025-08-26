#!/usr/bin/env python3
"""
Complete D455 Lawn Care System Launch File
Launches all components needed for autonomous lawn care
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('d455_lawn_care')
    
    # Configuration file paths
    config_file = os.path.join(pkg_dir, 'config', 'lawn_care_params.yaml')
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
    
    declare_enable_debug = DeclareLaunchArgument(
        'enable_debug',
        default_value='true',
        description='Enable debug visualizations'
    )
    
    declare_camera_serial = DeclareLaunchArgument(
        'camera_serial',
        default_value='',
        description='RealSense camera serial number'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Start RViz visualization'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    enable_debug = LaunchConfiguration('enable_debug')
    camera_serial = LaunchConfiguration('camera_serial')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # RealSense D455 Camera Node
    d455_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455_camera',
        namespace='d455',
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'serial_no': camera_serial,
                'device_type': 'd455',
                'enable_sync': True,
                'align_depth': True,
                'enable_pointcloud': True,
                'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
                'pointcloud_texture_index': 0,
                'allow_no_texture_points': True,
                'ordered_pc': False,
                
                # Stream configurations
                'enable_color': True,
                'color_width': 1280,
                'color_height': 720,
                'color_fps': 30,
                
                'enable_depth': True,
                'depth_width': 1280,
                'depth_height': 720,
                'depth_fps': 30,
                
                'enable_infra1': True,
                'enable_infra2': True,
                'infra_width': 1280,
                'infra_height': 720,
                'infra_fps': 30,
                
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
    
    # Grass Detection Node
    grass_detector_node = Node(
        package='d455_lawn_care',
        executable='grass_detector',
        name='grass_detector',
        namespace='lawn_care',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('~/image', '/d455/d455_camera/color/image_raw'),
            ('~/depth', '/d455/d455_camera/aligned_depth_to_color/image_raw'),
            ('~/camera_info', '/d455/d455_camera/color/camera_info'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )
    
    # Obstacle Detection Node
    obstacle_detector_node = Node(
        package='d455_lawn_care',
        executable='obstacle_detector',
        name='obstacle_detector',
        namespace='lawn_care',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('~/depth_image', '/d455/d455_camera/aligned_depth_to_color/image_raw'),
            ('~/color_image', '/d455/d455_camera/color/image_raw'),
            ('~/camera_info', '/d455/d455_camera/color/camera_info'),
            ('~/pointcloud', '/d455/d455_camera/depth/color/points'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )
    
    # Ground Plane Estimator Node (if available)
    ground_plane_node = Node(
        package='d455_lawn_care',
        executable='ground_plane_estimator',
        name='ground_plane_estimator',
        namespace='lawn_care',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('~/pointcloud', '/d455/d455_camera/depth/color/points'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        condition=IfCondition('false')  # Disabled by default, enable if node exists
    )
    
    # Static Transform Publishers for robot integration
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_d455_tf',
        arguments=[
            '0.2', '0', '0.3',           # x, y, z translation (camera 20cm forward, 30cm up)
            '0', '0.17', '0',            # roll, pitch, yaw (10 degree downward tilt)
            'base_link',                 # parent frame
            'd455_link'                  # child frame
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d455_optical_tf',
        arguments=[
            '0', '0', '0',
            '-0.5', '0.5', '-0.5', '0.5',  # Quaternion for optical frame orientation
            'd455_link',
            'd455_color_optical_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # RViz2 Visualization
    rviz_config_file = os.path.join(pkg_dir, 'config', 'lawn_care_visualization.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_rviz),
        output='screen'
    )
    
    # Image visualization nodes (for debugging)
    rqt_image_view_grass = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='grass_debug_viewer',
        arguments=['--ros-args', '-r', '__ns:=/lawn_care/grass_detector'],
        condition=IfCondition(enable_debug),
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_log_level)
    ld.add_action(declare_enable_debug)
    ld.add_action(declare_camera_serial)
    ld.add_action(declare_enable_rviz)
    
    # Add informational message
    ld.add_action(LogInfo(msg="Starting D455 Lawn Care System"))
    
    # Camera and transforms group
    camera_group = GroupAction([
        d455_camera_node,
        base_to_camera_tf,
        camera_optical_tf,
    ])
    
    # Lawn care processing group
    processing_group = GroupAction([
        grass_detector_node,
        obstacle_detector_node,
        ground_plane_node,
    ])
    
    # Visualization group
    visualization_group = GroupAction([
        rviz_node,
        rqt_image_view_grass,
    ])
    
    # Add groups to launch description
    ld.add_action(camera_group)
    ld.add_action(processing_group)
    ld.add_action(visualization_group)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()