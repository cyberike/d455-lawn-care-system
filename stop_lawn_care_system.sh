#!/bin/bash
# 🛑 One-Button Lawn Care System Shutdown
# Safely stops all lawn care system components

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${RED}🛑 STOPPING LAWN CARE SYSTEM${NC}"
echo "=============================="

# Function to kill process group and check
kill_process() {
    local name=$1
    local pattern=$2
    
    echo -e "${YELLOW}⏳ Stopping $name...${NC}"
    
    # Get PIDs matching pattern
    pids=$(pgrep -f "$pattern")
    
    if [ -n "$pids" ]; then
        # Kill processes
        echo "$pids" | xargs kill -TERM 2>/dev/null
        sleep 2
        
        # Force kill if still running
        remaining=$(pgrep -f "$pattern")
        if [ -n "$remaining" ]; then
            echo "$remaining" | xargs kill -KILL 2>/dev/null
            sleep 1
        fi
        
        # Check if stopped
        if ! pgrep -f "$pattern" > /dev/null; then
            echo -e "${GREEN}✅ $name stopped${NC}"
        else
            echo -e "${RED}⚠️  $name may still be running${NC}"
        fi
    else
        echo -e "${BLUE}ℹ️  $name not running${NC}"
    fi
}

# Stop all components
kill_process "D455 Camera" "realsense2_camera"
kill_process "D455 Driver" "d455"
kill_process "Grass Detector" "grass_detector"
kill_process "Obstacle Detector" "obstacle_detector"
kill_process "RViz" "rviz"

# Clean up log files
echo -e "${YELLOW}🧹 Cleaning up log files...${NC}"
rm -f /tmp/d455_camera.log
rm -f /tmp/grass_detector.log
rm -f /tmp/obstacle_detector.log
rm -f /tmp/rviz.log
echo -e "${GREEN}✅ Log files cleaned${NC}"

# Display remaining processes (if any)
echo ""
echo -e "${BLUE}🔍 Checking for remaining ROS processes...${NC}"
remaining_ros=$(pgrep -f "ros2\|realsense\|rviz\|grass_detector\|obstacle_detector" | wc -l)

if [ "$remaining_ros" -eq 0 ]; then
    echo -e "${GREEN}✅ All ROS processes stopped${NC}"
else
    echo -e "${YELLOW}⚠️  $remaining_ros ROS processes still running${NC}"
    echo "   Run 'ps aux | grep ros' to investigate"
fi

echo ""
echo -e "${GREEN}🛑 LAWN CARE SYSTEM SHUTDOWN COMPLETE${NC}"
echo ""
echo -e "${BLUE}💡 To restart the system, run:${NC}"
echo "   ./start_lawn_care_system.sh"