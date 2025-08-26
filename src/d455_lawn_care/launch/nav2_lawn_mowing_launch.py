#!/usr/bin/env python3
"""
Nav2 Lawn Mowing Launch File
Integrates D455 detection, costmap generation, and coverage path planning
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('d455_lawn_care')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration file paths
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_lawn_mowing_params.yaml')
    detection_params_file = os.path.join(pkg_dir, 'config', 'tuned_detection_params.yaml')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the Nav2 parameters file'
    )
    
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (optional)'
    )
    
    declare_robot_urdf = DeclareLaunchArgument(
        'robot_description_file',
        default_value=os.path.join(pkg_dir, 'urdf', 'mowing_robot.urdf'),
        description='Path to robot URDF file'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    robot_description_file = LaunchConfiguration('robot_description_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Set use_sim_time parameter globally
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # D455 Camera Node (reuse from existing launch)
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
                
                # Stream configurations
                'enable_color': True,
                'color_width': 848,
                'color_height': 480,
                'color_fps': 30,
                
                'enable_depth': True,
                'depth_width': 848,
                'depth_height': 480,
                'depth_fps': 30,
                
                # Outdoor optimizations
                'laser_power': 360,
                'exposure': 8000,
                'gain': 16,
                
                # Disable unnecessary streams
                'enable_fisheye': False,
                'enable_gyro': False,
                'enable_accel': False,
            }
        ],
        output='screen',
    )
    
    # Detection nodes with parameters
    grass_detector_node = Node(
        package='d455_lawn_care',
        executable='grass_detector_simple',
        name='simple_grass_detector',
        parameters=[detection_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    obstacle_detector_node = Node(
        package='d455_lawn_care',
        executable='obstacle_detector_simple',
        name='simple_obstacle_detector',
        parameters=[detection_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Costmap generator
    costmap_generator_node = Node(
        package='d455_lawn_care',
        executable='costmap_generator',
        name='lawn_care_costmap_generator',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'map_width': 50.0,
                'map_height': 50.0,
                'map_resolution': 0.1,
                'map_origin_x': -25.0,
                'map_origin_y': -25.0,
                'update_rate': 2.0,
                'grass_decay_rate': 0.95,
                'obstacle_decay_rate': 0.90,
            }
        ],
        output='screen'
    )
    
    # Coverage path planner
    coverage_planner_node = Node(
        package='d455_lawn_care',
        executable='coverage_path_planner',
        name='coverage_path_planner',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'mowing_width': 0.6,
                'overlap_ratio': 0.2,
                'min_grass_coverage': 0.1,
                'pattern_type': 'boustrophedon',
                'planning_frequency': 0.5,
            }
        ],
        output='screen'
    )
    
    # Robot state publisher (if robot URDF is provided)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': ''  # Will be loaded from file if provided
            }
        ],
        condition=IfCondition(robot_description_file),
        output='screen'
    )
    
    # Transform publishers for camera mounting
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_d455_tf',
        arguments=[
            '0.3', '0', '0.4',           # 30cm forward, 40cm up
            '0', '0.17', '0',            # 10 degree downward tilt
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
    
    # Nav2 bringup (full navigation stack)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_composition': use_composition,
            'map': map_yaml,
        }.items()
    )
    
    # RViz with lawn mowing configuration
    rviz_config_file = os.path.join(pkg_dir, 'config', 'lawn_mowing_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_robot_urdf)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_enable_rviz)
    
    # Set global parameters
    ld.add_action(set_use_sim_time)
    
    # Add informational message
    ld.add_action(LogInfo(msg="Starting D455 Lawn Care Navigation System"))
    
    # Sensor and detection group
    sensor_group = GroupAction([
        d455_camera_node,
        grass_detector_node,
        obstacle_detector_node,
        base_to_camera_tf,
        camera_optical_tf,
    ])
    
    # Navigation planning group  
    planning_group = GroupAction([
        costmap_generator_node,
        coverage_planner_node,
        robot_state_publisher_node,
    ])
    
    # Add all groups and nodes
    ld.add_action(sensor_group)
    ld.add_action(planning_group)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()