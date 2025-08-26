#!/usr/bin/env python3
"""
Gazebo Lawn Test Launch File
Launches minimal lawn simulation for parity testing
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # Get package directory
    pkg_dir = FindPackageShare('d455_lawn_care').find('d455_lawn_care')
    world_file = PathJoinSubstitution([pkg_dir, 'worlds', 'lawn_test.world'])
    
    return LaunchDescription([
        
        # Launch Gazebo with lawn world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
                 '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        
    ])