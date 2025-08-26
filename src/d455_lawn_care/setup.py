from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'd455_lawn_care'

setup(
    name='d455-lawn-care',  # Use dash format for setuptools compatibility
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yml'))),
        # URDF and meshes
        (os.path.join('share', package_name, 'urdf'), 
         glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'), 
         glob(os.path.join('meshes', '*'))),
        # Gazebo worlds
        (os.path.join('share', package_name, 'worlds'), 
         glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=[
        'setuptools',
        'opencv-python>=4.5.0',
        'numpy>=1.19.0',
        'scipy>=1.7.0',
        'matplotlib>=3.3.0',
        'scikit-learn>=1.0.0',
        'PyYAML>=5.4.0',
        'filterpy>=1.4.5',  # For Kalman filtering
        'shapely>=1.8.0',   # For polygon operations
        'pillow>=8.0.0',    # Image processing
        'scikit-image>=0.18.0',  # Advanced image processing
    ],
    zip_safe=True,
    maintainer='Lawn Care Developer',
    maintainer_email='you@example.com',
    description='ROS 2 package for Intel RealSense D455 lawn care applications',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Existing implemented nodes
            'grass_detector = d455_lawn_care.grass_detector:main',
            'obstacle_detector = d455_lawn_care.obstacle_detector:main',
            
            # Simple nodes with parameter support
            'grass_detector_simple = d455_lawn_care.grass_detector_simple:main',
            'obstacle_detector_simple = d455_lawn_care.obstacle_detector_simple:main',
            
            # Nav2 integration nodes
            'costmap_generator = d455_lawn_care.costmap_generator:main',
            'coverage_path_planner = d455_lawn_care.coverage_path_planner:main',
            'simple_coverage_planner = d455_lawn_care.simple_coverage_planner:main',
            
            # Future nodes (commented out until implemented)
            # 'ground_plane_estimator = d455_lawn_care.ground_plane_estimator:main',
            # 'lawn_navigation = d455_lawn_care.lawn_navigation:main',
            # 'mowing_pattern_planner = d455_lawn_care.mowing_pattern_planner:main',
            # 'camera_calibrator = d455_lawn_care.camera_calibrator:main',
            # 'grass_height_analyzer = d455_lawn_care.grass_height_analyzer:main',
            # 'weather_monitor = d455_lawn_care.weather_monitor:main',
            # 'battery_monitor = d455_lawn_care.battery_monitor:main',
        ],
    },
    python_requires='>=3.8',
)