#!/bin/bash

# Complete Recording Setup for D455 Lawn Care System
# Records ROS2 bag + videos + raw images for Gazebo simulation

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== D455 Lawn Care Complete Recording System ===${NC}"

# Check if system is running
if ! ros2 topic list | grep -q "d455_camera"; then
    echo -e "${RED}Error: D455 camera not detected. Please start the camera first.${NC}"
    exit 1
fi

if ! ros2 topic list | grep -q "grass_detector"; then
    echo -e "${RED}Error: Lawn care nodes not detected. Please start grass/obstacle detectors.${NC}"
    exit 1
fi

# Configuration
RECORDING_DIR="$HOME/lawn_care_recordings"
DURATION=${1:-120}  # Default 2 minutes
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
SESSION_DIR="$RECORDING_DIR/complete_session_$TIMESTAMP"

echo -e "${BLUE}Recording Duration: $DURATION seconds${NC}"
echo -e "${BLUE}Output Directory: $SESSION_DIR${NC}"

# Create session directory
mkdir -p "$SESSION_DIR"

# List of processes to start
declare -a PROCESSES=()

echo -e "${YELLOW}Starting recording processes...${NC}"

# 1. Start ROS2 bag recording
echo -e "${BLUE}1. Starting ROS2 bag recording...${NC}"
(
    cd "$SESSION_DIR"
    mkdir -p rosbag
    cd rosbag
    
    ros2 bag record \
        --storage mcap \
        --compression-mode file \
        --compression-format zstd \
        --max-bag-duration 0 \
        --output "lawn_care_$TIMESTAMP" \
        /d455/d455_camera/color/image_raw \
        /d455/d455_camera/aligned_depth_to_color/image_raw \
        /d455/d455_camera/color/camera_info \
        /d455/d455_camera/depth/camera_info \
        /simple_grass_detector/grass_coverage_percentage \
        /simple_grass_detector/grass_status \
        /simple_grass_detector/debug_image \
        /simple_grass_detector/grass_mask \
        /simple_obstacle_detector/obstacle_count \
        /simple_obstacle_detector/obstacle_status \
        /simple_obstacle_detector/obstacle_markers \
        /simple_obstacle_detector/debug_image \
        /tf /tf_static
) &

ROSBAG_PID=$!
PROCESSES+=($ROSBAG_PID)

sleep 2

# 2. Start video recording
echo -e "${BLUE}2. Starting video recording...${NC}"
python3 /home/cyberike/lawn_care_ws/record_video_streams.py \
    --duration $DURATION \
    --output "$SESSION_DIR/videos" \
    --topics \
        /d455/d455_camera/color/image_raw \
        /d455/d455_camera/aligned_depth_to_color/image_raw \
        /simple_grass_detector/debug_image \
        /simple_grass_detector/grass_mask \
        /simple_obstacle_detector/debug_image &

VIDEO_PID=$!
PROCESSES+=($VIDEO_PID)

sleep 2

# 3. Start comprehensive data recorder
echo -e "${BLUE}3. Starting comprehensive data recorder...${NC}"
python3 /home/cyberike/lawn_care_ws/record_lawn_care_data.py &

DATA_PID=$!
PROCESSES+=($DATA_PID)

echo -e "${GREEN}All recording processes started!${NC}"
echo -e "${YELLOW}Recording for $DURATION seconds...${NC}"

# Function to cleanup processes
cleanup() {
    echo -e "\n${YELLOW}Stopping recording processes...${NC}"
    for pid in "${PROCESSES[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            kill -TERM $pid
            sleep 2
            if kill -0 $pid 2>/dev/null; then
                kill -KILL $pid
            fi
        fi
    done
    
    echo -e "${GREEN}Recording complete! Data saved to:${NC}"
    echo -e "${BLUE}$SESSION_DIR${NC}"
    
    # Create session summary
    cat > "$SESSION_DIR/session_info.txt" << EOF
D455 Lawn Care Recording Session
===============================
Session ID: $TIMESTAMP
Duration: $DURATION seconds
Directory: $SESSION_DIR

Contents:
- rosbag/: ROS2 bag files (MCAP format, compressed)
- videos/: MP4 video files of all streams  
- lawn_care_session_*/: Raw images and additional videos

Usage for Gazebo Simulation:
1. Playback: ros2 bag play rosbag/lawn_care_$TIMESTAMP
2. Extract frames from videos for texture generation
3. Use grass detection patterns for world validation
4. Import obstacle data for realistic world creation

Recorded: $(date)
EOF

    echo -e "${GREEN}Session summary: $SESSION_DIR/session_info.txt${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for duration or user interrupt
sleep $DURATION

# Cleanup automatically after duration
cleanup