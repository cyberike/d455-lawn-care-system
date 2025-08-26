# 🌱 D455 Lawn Care System - One-Button Scripts

Quick reference for the automated lawn care system scripts.

## 🚀 Scripts Available

### 1. **System Startup** - `./start_lawn_care_system.sh`
**One-button startup of complete system**

**What it does:**
- ✅ Starts D455 camera with depth streams (10Hz color + depth)
- ✅ Launches grass detector (HSV-based detection)
- ✅ Launches obstacle detector (depth-based detection)  
- ✅ Opens RViz for visualization
- ✅ Validates all topics are publishing
- ✅ Displays system status and process IDs

**Usage:**
```bash
./start_lawn_care_system.sh
```

**Expected Output:**
```
🌱 D455 LAWN CARE SYSTEM STARTUP
=================================
✅ Changed to workspace directory
✅ Workspace sourced
✅ Cleanup complete
✅ D455 camera launched (PID: 12345)
✅ Color stream available
✅ Depth stream available
✅ Grass detector started (PID: 12346)
✅ Obstacle detector started (PID: 12347)
✅ RViz started (PID: 12348)

🎉 LAWN CARE SYSTEM READY!
```

---

### 2. **System Shutdown** - `./stop_lawn_care_system.sh`
**Clean shutdown of all components**

**What it does:**
- 🛑 Stops D455 camera processes
- 🛑 Stops detection nodes
- 🛑 Stops RViz
- 🧹 Cleans up log files
- 🔍 Verifies all processes stopped

**Usage:**
```bash
./stop_lawn_care_system.sh
```

---

### 3. **Parity Test** - `./run_parity_test.sh [duration]`
**Complete performance analysis vs simulation**

**What it does:**
- 🎯 Auto-starts system if not running
- 📊 Collects performance data (camera rates, detection accuracy)
- 🔍 Analyzes against expected simulation performance
- 📋 Generates comprehensive report with grades
- 💡 Provides optimization recommendations

**Usage:**
```bash
# Default 60-second test
./run_parity_test.sh

# Custom duration (e.g., 120 seconds)
./run_parity_test.sh 120
```

**Expected Output:**
```
🎯 GAZEBO PARITY TEST RUNNER
============================
📊 PRE-TEST SYSTEM STATUS
Color: 25.3 Hz
Depth: 24.1 Hz
Grass: 10.0 Hz
Obstacles: 0.0 Hz

🚀 STARTING PARITY TEST
⏳ Test running...
✅ Parity test completed

📊 RUNNING ANALYSIS
🏆 OVERALL PARITY SCORE: 36.0/100 (Grade F)
```

---

## 📋 System Requirements

**Before Running Scripts:**
1. ✅ RealSense D455 connected via USB 3.0
2. ✅ Workspace built: `colcon build --packages-select d455_lawn_care`
3. ✅ RealSense drivers installed: `~/realsense_launch/source_ros_relaxed.sh`

---

## 🔧 Troubleshooting

### **Camera Not Starting**
```bash
# Check USB connection
lsusb | grep Intel

# Check RealSense install
~/realsense_launch/source_ros_relaxed.sh
```

### **Low Frame Rates (<20Hz)**
- ⚠️ Check USB 3.0 connection (not USB 2.0)
- ⚠️ Check system CPU load
- ⚠️ Try different USB port

### **Grass Detection Low (<5%)**
- 🌱 Test with actual grass (not indoor surfaces)
- 🌱 Retune HSV parameters in config files
- 🌱 Check lighting conditions

### **No Obstacle Detection**
- 🚧 Ensure depth stream is working (`ros2 topic hz /d455/d455/depth/image_rect_raw`)
- 🚧 Place objects within 0.3-3m range
- 🚧 Check obstacle detector logs

---

## 📁 Files Created

**Log Files:**
- `/tmp/d455_camera.log` - Camera driver logs
- `/tmp/grass_detector.log` - Grass detection logs  
- `/tmp/obstacle_detector.log` - Obstacle detection logs
- `/tmp/rviz.log` - RViz logs

**Test Results:**
- `/tmp/parity_test_real_*.json` - Raw test data
- `/tmp/parity_analysis_*.json` - Analysis results
- `/tmp/parity_test_output.log` - Test execution log

---

## 🎯 Quick Commands

**Monitor System:**
```bash
# Check all topics
ros2 topic list

# Monitor grass detection
ros2 topic echo /simple_grass_detector/grass_coverage_percentage

# Monitor obstacles  
ros2 topic echo /simple_obstacle_detector/obstacle_count

# Check camera rates
ros2 topic hz /d455/d455/color/image_raw
ros2 topic hz /d455/d455/depth/image_rect_raw
```

**Development:**
```bash
# Build workspace
colcon build --packages-select d455_lawn_care

# Source workspace
source install/setup.bash

# Manual camera launch
ros2 launch realsense2_camera rs_launch.py enable_depth:=true
```

---

## 🚀 Typical Workflow

1. **Start System:**
   ```bash
   ./start_lawn_care_system.sh
   ```

2. **Test Performance:**
   ```bash
   ./run_parity_test.sh 120
   ```

3. **Tune Parameters** (based on test results)

4. **Stop System:**
   ```bash
   ./stop_lawn_care_system.sh
   ```

---

## 📊 Performance Expectations

**Good Performance:**
- 📷 Camera: 25-30 Hz (color and depth)
- 🌱 Grass Detection: 15-25 Hz, >50% accuracy in grass scenes
- 🚧 Obstacle Detection: 15-25 Hz, detecting objects 0.3-3m range
- 🎯 Parity Score: >60/100 (Grade C+)

**Current Status:**
- 📷 Camera: 10 Hz (needs USB/power optimization)
- 🌱 Grass: 0.2% accuracy (needs HSV tuning)
- 🚧 Obstacles: 0% (indoor environment, no obstacles)
- 🎯 Parity Score: 36/100 (Grade F - needs optimization)

---

*Created: 2025-08-25*  
*System: D455 Lawn Care with ROS 2 Humble*