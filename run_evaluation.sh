#!/bin/bash

# D455 Lawn Care Detection Performance Evaluation Script
# Runs bag playback with detection nodes and evaluates performance

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== D455 Lawn Care Detection Evaluation ===${NC}"

# Configuration
BAG_PATH=${1:-"lawn_bag"}
EVALUATION_DURATION=${2:-30}
OUTPUT_DIR=${3:-"evaluation_$(date +%Y%m%d_%H%M%S)"}

echo -e "${BLUE}Configuration:${NC}"
echo -e "  Bag path: ${BAG_PATH}"
echo -e "  Duration: ${EVALUATION_DURATION} seconds"
echo -e "  Output: ${OUTPUT_DIR}"
echo ""

# Check if bag exists
if [ ! -d "$BAG_PATH" ] && [ ! -f "$BAG_PATH" ]; then
    echo -e "${RED}Error: Bag file '$BAG_PATH' not found!${NC}"
    exit 1
fi

# Source ROS environment
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}✓${NC} ROS workspace sourced"
else
    echo -e "${YELLOW}Warning: install/setup.bash not found, using system ROS${NC}"
fi

# Array to track background processes
declare -a PIDS=()

# Function to cleanup processes
cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    for pid in "${PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            kill -TERM $pid 2>/dev/null
            sleep 1
            if kill -0 $pid 2>/dev/null; then
                kill -KILL $pid 2>/dev/null
            fi
        fi
    done
    echo -e "${GREEN}✓${NC} Cleanup complete"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo -e "${BLUE}Starting evaluation processes...${NC}"

# 1. Start bag playback with loop and clock
echo -e "${BLUE}1. Starting bag playback...${NC}"
ros2 bag play "$BAG_PATH" --clock --loop &
BAG_PID=$!
PIDS+=($BAG_PID)
sleep 2

# 2. Start grass detector with parameters
echo -e "${BLUE}2. Starting grass detector...${NC}"
if [ -f "install/d455_lawn_care/bin/grass_detector_simple" ]; then
    install/d455_lawn_care/bin/grass_detector_simple \
        --ros-args --params-file src/d455_lawn_care/config/grass_detector_params.yaml &
    GRASS_PID=$!
    PIDS+=($GRASS_PID)
    sleep 1
else
    echo -e "${RED}Error: grass_detector_simple executable not found!${NC}"
    cleanup
    exit 1
fi

# 3. Start obstacle detector (if available)
echo -e "${BLUE}3. Starting obstacle detector...${NC}"
if [ -f "install/d455_lawn_care/bin/obstacle_detector_simple" ]; then
    install/d455_lawn_care/bin/obstacle_detector_simple \
        --ros-args --params-file src/d455_lawn_care/config/tuned_detection_params.yaml &
    OBSTACLE_PID=$!
    PIDS+=($OBSTACLE_PID)
    sleep 1
else
    echo -e "${YELLOW}Warning: obstacle_detector_simple not found, skipping${NC}"
fi

# 4. Wait for nodes to initialize
echo -e "${BLUE}4. Waiting for nodes to initialize...${NC}"
sleep 3

# 5. Check if topics are available
echo -e "${BLUE}5. Checking topic availability...${NC}"
TIMEOUT=10
for topic in "/simple_grass_detector/grass_coverage_percentage"; do
    echo -n "  Waiting for $topic... "
    if timeout $TIMEOUT ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        echo -e "${GREEN}✓${NC}"
    else
        echo -e "${RED}✗ (timeout)${NC}"
        echo -e "${YELLOW}Warning: Topic $topic not available${NC}"
    fi
done

# 6. Start evaluation
echo -e "${BLUE}6. Starting performance evaluation...${NC}"
python3 evaluate_detection_performance.py \
    --duration $EVALUATION_DURATION \
    --output "$OUTPUT_DIR" &
EVAL_PID=$!
PIDS+=($EVAL_PID)

# 7. Monitor evaluation progress
echo -e "${GREEN}✓${NC} Evaluation started (PID: $EVAL_PID)"
echo -e "${YELLOW}Evaluation running for ${EVALUATION_DURATION} seconds...${NC}"
echo -e "Press Ctrl+C to stop early"

# Wait for evaluation to complete or be interrupted
wait $EVAL_PID
EVAL_RESULT=$?

# Check evaluation result
if [ $EVAL_RESULT -eq 0 ]; then
    echo -e "\n${GREEN}✓ Evaluation completed successfully!${NC}"
    
    # Display results if available
    if [ -d "$OUTPUT_DIR" ]; then
        echo -e "\n${BLUE}Results summary:${NC}"
        if [ -f "$OUTPUT_DIR/performance_report.txt" ]; then
            echo -e "${BLUE}Performance Report:${NC}"
            echo "$(head -20 "$OUTPUT_DIR/performance_report.txt")"
            echo ""
            echo -e "📊 Full report: ${BLUE}$OUTPUT_DIR/performance_report.txt${NC}"
            
            if [ -f "$OUTPUT_DIR/performance_plots.png" ]; then
                echo -e "📈 Visualizations: ${BLUE}$OUTPUT_DIR/performance_plots.png${NC}"
            fi
        fi
        
        echo -e "\n${GREEN}All results saved to: ${OUTPUT_DIR}${NC}"
    fi
else
    echo -e "\n${RED}✗ Evaluation failed or was interrupted${NC}"
fi

# Cleanup
cleanup