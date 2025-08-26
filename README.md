# 🌱 D455 Lawn Care System

**Intel RealSense D455 autonomous lawn care system with ROS 2 Humble**

Complete computer vision pipeline for grass detection, obstacle avoidance, and coverage path planning using depth and color cameras.

## 📁 Package Structure

```
~/lawn_care_ws/
├── src/d455_lawn_care/           # Main package directory
│   ├── d455_lawn_care/           # Python package
│   │   ├── grass_detector.py     # Grass detection node
│   │   └── obstacle_detector.py  # Obstacle detection node
│   ├── msg/                      # Custom message definitions
│   │   ├── GrassDetection.msg    # Grass analysis results
│   │   ├── Obstacle.msg          # Individual obstacle data
│   │   ├── ObstacleArray.msg     # Multiple obstacles
│   │   └── LawnStatus.msg        # Overall system status
│   ├── srv/                      # Service definitions
│   │   ├── CalibrateGrassDetection.srv  # Calibration service
│   │   └── SetMowingPattern.srv  # Pattern planning service
│   ├── launch/                   # Launch files
│   │   └── lawn_care_system.launch.py  # Complete system launch
│   ├── config/                   # Configuration files
│   │   └── lawn_care_params.yaml # Tuned parameters
│   └── package.xml              # Package dependencies
```

## 🚀 Quick Start

### 1. Build the Workspace
```bash
cd ~/lawn_care_ws
colcon build --packages-select d455_lawn_care
source install/setup.bash
```

### 2. Launch D455 Camera (already running)
```bash
# Your D455 is already streaming with optimal settings
ros2 topic list | grep d455
```

### 3. Launch Lawn Care System
```bash
source ~/lawn_care_ws/install/setup.bash
ros2 launch d455_lawn_care lawn_care_system.launch.py
```

### 4. Run Individual Nodes
```bash
# Grass detection
python3 ~/lawn_care_ws/src/d455_lawn_care/d455_lawn_care/grass_detector.py

# Obstacle detection  
python3 ~/lawn_care_ws/src/d455_lawn_care/d455_lawn_care/obstacle_detector.py
```

## 🌱 Features

### Grass Detection Node
- **Real-time grass analysis** using HSV color filtering
- **Height measurement** using RGB-D depth data
- **Coverage percentage** calculation
- **Cutting recommendations** based on height thresholds
- **Auto-calibration** service for different grass types
- **Debug visualizations** with grass mask overlay

### Obstacle Detection Node  
- **3D obstacle detection** using depth clustering
- **Obstacle classification** (tree, rock, furniture, etc.)
- **Persistent tracking** with unique IDs
- **Safety zone calculation** with danger levels
- **RViz visualization** with colored bounding boxes
- **Configurable detection parameters**

### Custom Messages
- `GrassDetection.msg` - Complete grass analysis data
- `Obstacle.msg` - Individual obstacle properties
- `ObstacleArray.msg` - Multiple obstacles with statistics
- `LawnStatus.msg` - Overall system health and status

### Services
- `CalibrateGrassDetection.srv` - Adaptive grass color calibration
- `SetMowingPattern.srv` - Autonomous mowing path planning

## 🔧 Configuration

All parameters are tuned for D455 lawn care in `config/lawn_care_params.yaml`:

### Grass Detection Parameters
```yaml
grass_detector:
  ros__parameters:
    # HSV color range (outdoor optimized)
    grass_color_hue_min: 40      # Green hue lower bound
    grass_color_hue_max: 80      # Green hue upper bound
    
    # Height thresholds
    min_grass_height: 0.02       # 2cm minimum
    cutting_threshold: 0.08      # 8cm cutting trigger
    
    # Performance
    detection_frequency: 10.0    # 10 Hz detection rate
```

### Obstacle Detection Parameters
```yaml
obstacle_detector:
  ros__parameters:
    # Detection ranges
    min_obstacle_height: 0.10    # 10cm minimum height
    max_detection_distance: 10.0 # 10m maximum range
    
    # Safety zones
    safety_buffer_distance: 0.5  # 50cm safety buffer
    danger_zone_distance: 1.0    # 1m danger zone
```

## 📊 Topics Published

### From Grass Detector:
- `/lawn_care/grass_detector/grass_detection` - Grass analysis results
- `/lawn_care/grass_detector/debug_image` - Visualization with overlay
- `/lawn_care/grass_detector/grass_mask` - Binary grass mask

### From Obstacle Detector:
- `/lawn_care/obstacle_detector/obstacles` - Detected obstacles array
- `/lawn_care/obstacle_detector/obstacle_markers` - RViz visualization
- `/lawn_care/obstacle_detector/debug_image` - Debug visualization

### From D455 Camera:
- `/d455/d455_camera/color/image_raw` - RGB camera feed
- `/d455/d455_camera/aligned_depth_to_color/image_raw` - Aligned depth
- `/d455/d455_camera/depth/color/points` - Point cloud data

## 🎯 Current Status

✅ **Package Built Successfully:**
- ROS 2 workspace created and configured
- Custom messages and services defined  
- Python nodes with OpenCV dependencies
- Launch files for system integration
- Configuration parameters optimized for D455

✅ **D455 Camera Active:**
- Streaming at 1280x720 @ 30fps
- Outdoor-optimized settings applied
- All depth processing filters active
- Perfect integration with ROS 2 topics

✅ **Ready for Integration:**
- Grass detection algorithms implemented
- Obstacle detection and tracking ready
- Calibration services available
- Debug visualizations enabled

## 🔄 Next Steps

1. **Test Grass Detection:**
   ```bash
   python3 ~/lawn_care_ws/src/d455_lawn_care/d455_lawn_care/grass_detector.py
   ```

2. **Test Obstacle Detection:**
   ```bash  
   python3 ~/lawn_care_ws/src/d455_lawn_care/d455_lawn_care/obstacle_detector.py
   ```

3. **Launch Complete System:**
   ```bash
   ros2 launch d455_lawn_care lawn_care_system.launch.py
   ```

4. **Calibrate for Your Grass:**
   ```bash
   ros2 service call /lawn_care/grass_detector/calibrate_grass_detection d455_lawn_care/srv/CalibrateGrassDetection "{auto_calibrate: true}"
   ```

## 🛠️ Dependencies Included

- **OpenCV** for computer vision processing
- **NumPy** for numerical computations  
- **SciPy** for advanced mathematical functions
- **scikit-learn** for machine learning features
- **Matplotlib** for data visualization
- **cv_bridge** for ROS-OpenCV integration
- **tf2_ros** for coordinate transformations

## 📝 Notes

- The package is built as a pure Python package for easier development
- All parameters are configurable via YAML files
- Debug visualizations help with tuning and debugging
- Custom messages provide structured data for lawn care applications
- Services enable dynamic reconfiguration and calibration