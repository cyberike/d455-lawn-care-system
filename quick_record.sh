#!/bin/bash

# Quick ROS2 Bag Recording for D455 Lawn Care
# Simple script to record essential topics for Gazebo simulation

set -e

# Configuration
RECORDING_DIR="$HOME/lawn_care_recordings"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
SESSION_DIR="$RECORDING_DIR/session_$TIMESTAMP"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== D455 Lawn Care Quick Recording ===${NC}"
echo -e "${BLUE}Session: $SESSION_DIR${NC}"

# Create directories
mkdir -p "$SESSION_DIR/rosbag"
mkdir -p "$SESSION_DIR/videos"

# Essential topics for Gazebo simulation
TOPICS=(
    # Core D455 data
    "/d455/d455_camera/color/image_raw"
    "/d455/d455_camera/aligned_depth_to_color/image_raw"
    "/d455/d455_camera/color/camera_info"
    "/d455/d455_camera/depth/camera_info"
    
    # Lawn care processing results
    "/simple_grass_detector/grass_coverage_percentage"
    "/simple_grass_detector/grass_status"
    "/simple_grass_detector/debug_image"
    "/simple_grass_detector/grass_mask"
    
    "/simple_obstacle_detector/obstacle_count"
    "/simple_obstacle_detector/obstacle_status"
    "/simple_obstacle_detector/obstacle_markers"
    "/simple_obstacle_detector/debug_image"
    
    # Transforms for robot simulation
    "/tf"
    "/tf_static"
)

echo -e "${BLUE}Recording ${#TOPICS[@]} topics...${NC}"

# Start recording
cd "$SESSION_DIR/rosbag"

echo -e "${YELLOW}Starting ROS2 bag recording...${NC}"
echo "Press Ctrl+C to stop"

ros2 bag record \
    --storage mcap \
    --compression-mode file \
    --compression-format zstd \
    --max-bag-size 1000000000 \
    --output "lawn_care_bag_$TIMESTAMP" \
    "${TOPICS[@]}"

echo -e "${GREEN}Recording saved to: $SESSION_DIR${NC}"

# Create recording info file
cat > "$SESSION_DIR/recording_info.txt" << EOF
D455 Lawn Care Recording Session
================================
Timestamp: $TIMESTAMP
Directory: $SESSION_DIR
Topics Recorded: ${#TOPICS[@]}

Topics:
$(printf '%s\n' "${TOPICS[@]}")

Usage for Gazebo:
1. Play back bag: ros2 bag play $SESSION_DIR/rosbag/lawn_care_bag_$TIMESTAMP
2. Launch Gazebo with lawn world
3. Remap topics to match Gazebo simulation

Recording completed: $(date)
EOF

echo -e "${GREEN}Recording info saved to: $SESSION_DIR/recording_info.txt${NC}"