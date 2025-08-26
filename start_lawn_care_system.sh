#!/bin/bash
# 🌱 One-Button Lawn Care System Startup
# Starts complete D455 lawn care system with proper configuration

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

echo -e "${GREEN}🌱 D455 LAWN CARE SYSTEM STARTUP${NC}"
echo "================================="

# Function to check if command succeeded
check_status() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ $1${NC}"
    else
        echo -e "${RED}❌ $1 FAILED${NC}"
        exit 1
    fi
}

# Function to wait for ROS topics
wait_for_topics() {
    local topic=$1
    local timeout=${2:-30}
    echo -e "${YELLOW}⏳ Waiting for $topic...${NC}"
    
    for i in $(seq 1 $timeout); do
        if ros2 topic list | grep -q "$topic"; then
            echo -e "${GREEN}✅ $topic available${NC}"
            return 0
        fi
        sleep 1
    done
    
    echo -e "${RED}❌ Timeout waiting for $topic${NC}"
    return 1
}

# Function to kill existing processes
cleanup_existing() {
    echo -e "${YELLOW}🧹 Cleaning up existing processes...${NC}"
    
    # Kill existing RealSense processes
    pkill -f realsense2_camera > /dev/null 2>&1
    pkill -f d455 > /dev/null 2>&1
    
    # Kill detection processes
    pkill -f grass_detector > /dev/null 2>&1
    pkill -f obstacle_detector > /dev/null 2>&1
    
    # Kill RViz
    pkill -f rviz > /dev/null 2>&1
    
    sleep 2
    echo -e "${GREEN}✅ Cleanup complete${NC}"
}

# Change to workspace directory
cd ~/lawn_care_ws
check_status "Changed to workspace directory"

# Source workspace
echo -e "${BLUE}📦 Sourcing workspace...${NC}"
source install/setup.bash
check_status "Workspace sourced"

# Cleanup existing processes
cleanup_existing

# Start D455 camera with depth streams
echo -e "${BLUE}📷 Starting D455 camera with depth streams...${NC}"
. ~/realsense_launch/source_ros_relaxed.sh
nohup ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=true \
    enable_color:=true \
    align_depth:=true \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_gyro:=false \
    enable_accel:=false \
    camera_name:=d455 \
    camera_namespace:=d455 \
    > /tmp/d455_camera.log 2>&1 &

D455_PID=$!
sleep 15
check_status "D455 camera launched (PID: $D455_PID)"

# Wait for camera topics with longer timeout
wait_for_topics "/d455/d455/color/image_raw" 30
wait_for_topics "/d455/d455/depth/image_rect_raw" 30

# Start grass detector
echo -e "${BLUE}🌱 Starting grass detector...${NC}"
nohup install/d455_lawn_care/bin/grass_detector_simple > /tmp/grass_detector.log 2>&1 &
GRASS_PID=$!
sleep 3
check_status "Grass detector started (PID: $GRASS_PID)"

# Start obstacle detector
echo -e "${BLUE}🚧 Starting obstacle detector...${NC}"
nohup install/d455_lawn_care/bin/obstacle_detector_simple > /tmp/obstacle_detector.log 2>&1 &
OBSTACLE_PID=$!
sleep 3
check_status "Obstacle detector started (PID: $OBSTACLE_PID)"

# Wait for detection topics
wait_for_topics "/simple_grass_detector/grass_coverage_percentage" 10
wait_for_topics "/simple_obstacle_detector/obstacle_count" 10

# Optional: Start RViz for visualization
echo -e "${PURPLE}🖥️  Starting RViz for visualization...${NC}"
nohup rviz2 > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 2
check_status "RViz started (PID: $RVIZ_PID)"

# Display system status
echo ""
echo -e "${GREEN}🎉 LAWN CARE SYSTEM READY!${NC}"
echo "=========================="
echo -e "${BLUE}📊 System Status:${NC}"

# Check topic rates
echo -e "${YELLOW}📷 Camera Streams:${NC}"
timeout 3s ros2 topic hz /d455/d455/color/image_raw --once 2>/dev/null | grep "average rate" | head -1 || echo "  Color: Starting up..."
timeout 3s ros2 topic hz /d455/d455/depth/image_rect_raw --once 2>/dev/null | grep "average rate" | head -1 || echo "  Depth: Starting up..."

echo -e "${YELLOW}🧠 Detection Streams:${NC}"
timeout 3s ros2 topic hz /simple_grass_detector/grass_coverage_percentage --once 2>/dev/null | grep "average rate" | head -1 || echo "  Grass: Starting up..."
timeout 3s ros2 topic hz /simple_obstacle_detector/obstacle_count --once 2>/dev/null | grep "average rate" | head -1 || echo "  Obstacles: Starting up..."

echo ""
echo -e "${GREEN}📋 Process IDs:${NC}"
echo "  D455 Camera: $D455_PID"
echo "  Grass Detector: $GRASS_PID" 
echo "  Obstacle Detector: $OBSTACLE_PID"
echo "  RViz: $RVIZ_PID"

echo ""
echo -e "${BLUE}📁 Log Files:${NC}"
echo "  Camera: /tmp/d455_camera.log"
echo "  Grass: /tmp/grass_detector.log"
echo "  Obstacles: /tmp/obstacle_detector.log"
echo "  RViz: /tmp/rviz.log"

echo ""
echo -e "${PURPLE}🔧 Useful Commands:${NC}"
echo "  Check topics: ros2 topic list"
echo "  Monitor grass: ros2 topic echo /simple_grass_detector/grass_coverage_percentage"
echo "  Monitor obstacles: ros2 topic echo /simple_obstacle_detector/obstacle_count"
echo "  Stop system: ./stop_lawn_care_system.sh"

echo ""
echo -e "${GREEN}✅ System startup complete! Ready for testing.${NC}"