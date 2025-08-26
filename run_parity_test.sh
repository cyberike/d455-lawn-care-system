#!/bin/bash
# 🎯 One-Button Parity Test Runner  
# Runs complete gazebo parity analysis with proper system setup

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Test duration (default 60 seconds)
TEST_DURATION=${1:-60}

echo -e "${PURPLE}🎯 GAZEBO PARITY TEST RUNNER${NC}"
echo "============================="
echo -e "${BLUE}Test Duration: ${TEST_DURATION}s${NC}"
echo ""

# Function to check if command succeeded
check_status() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ $1${NC}"
    else
        echo -e "${RED}❌ $1 FAILED${NC}"
        exit 1
    fi
}

# Function to wait for system ready
wait_for_system() {
    echo -e "${YELLOW}⏳ Waiting for system to be ready...${NC}"
    
    # Check for required topics
    local topics=("/d455/d455/color/image_raw" "/d455/d455/depth/image_rect_raw" "/simple_grass_detector/grass_coverage_percentage" "/simple_obstacle_detector/obstacle_count")
    
    for topic in "${topics[@]}"; do
        echo -e "${BLUE}   Checking $topic...${NC}"
        timeout 15s sh -c "until ros2 topic list | grep -q '$topic'; do sleep 1; done"
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}   ✅ $topic ready${NC}"
        else
            echo -e "${RED}   ❌ $topic not available${NC}"
            return 1
        fi
    done
    
    echo -e "${GREEN}✅ System ready for testing${NC}"
    return 0
}

# Change to workspace directory
cd ~/lawn_care_ws
check_status "Changed to workspace directory"

# Source workspace
echo -e "${BLUE}📦 Sourcing workspace...${NC}"
source install/setup.bash
check_status "Workspace sourced"

# Check if system is running
echo -e "${BLUE}🔍 Checking system status...${NC}"
if ! ros2 topic list | grep -q "/d455/d455/color/image_raw"; then
    echo -e "${YELLOW}⚠️  System not running. Starting automatically...${NC}"
    ./start_lawn_care_system.sh
    sleep 10
fi

# Wait for system to be ready
wait_for_system
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ System not ready. Please check the lawn care system.${NC}"
    echo -e "${BLUE}💡 Try running: ./start_lawn_care_system.sh${NC}"
    exit 1
fi

# Display pre-test status
echo ""
echo -e "${GREEN}📊 PRE-TEST SYSTEM STATUS${NC}"
echo "========================"

echo -e "${BLUE}📷 Camera Status:${NC}"
color_rate=$(timeout 3s ros2 topic hz /d455/d455/color/image_raw 2>/dev/null | grep "average rate" | awk '{print $3}' | head -1)
depth_rate=$(timeout 3s ros2 topic hz /d455/d455/depth/image_rect_raw 2>/dev/null | grep "average rate" | awk '{print $3}' | head -1)
echo "   Color: ${color_rate:-Starting...} Hz"
echo "   Depth: ${depth_rate:-Starting...} Hz"

echo -e "${BLUE}🧠 Detection Status:${NC}"
grass_rate=$(timeout 3s ros2 topic hz /simple_grass_detector/grass_coverage_percentage 2>/dev/null | grep "average rate" | awk '{print $3}' | head -1)
obstacle_rate=$(timeout 3s ros2 topic hz /simple_obstacle_detector/obstacle_count 2>/dev/null | grep "average rate" | awk '{print $3}' | head -1)
echo "   Grass: ${grass_rate:-Starting...} Hz"
echo "   Obstacles: ${obstacle_rate:-Starting...} Hz"

# Get current detection values
echo -e "${BLUE}🔢 Current Detection Values:${NC}"
grass_value=$(timeout 2s ros2 topic echo /simple_grass_detector/grass_coverage_percentage --once 2>/dev/null | grep "data:" | awk '{print $2}')
obstacle_value=$(timeout 2s ros2 topic echo /simple_obstacle_detector/obstacle_count --once 2>/dev/null | grep "data:" | awk '{print $2}')
echo "   Grass Coverage: ${grass_value:-N/A}%"
echo "   Obstacle Count: ${obstacle_value:-N/A}"

echo ""
echo -e "${PURPLE}🚀 STARTING PARITY TEST${NC}"
echo "======================="
echo -e "${YELLOW}Duration: ${TEST_DURATION}s${NC}"
echo -e "${YELLOW}Expected completion: $(date -d "+${TEST_DURATION} seconds" "+%H:%M:%S")${NC}"
echo ""

# Run the parity test
echo -e "${BLUE}🎯 Running parity test...${NC}"

# Create a modified test script with custom duration
cat > /tmp/run_parity_test_custom.py << EOF
#!/usr/bin/env python3
import sys
import os
sys.path.append('${PWD}')

# Import the original test
exec(open('test_gazebo_parity.py').read().replace('self.test_duration = 120.0', 'self.test_duration = ${TEST_DURATION}.0'))
EOF

chmod +x /tmp/run_parity_test_custom.py

# Run the test with timeout
timeout $((TEST_DURATION + 30))s python3 /tmp/run_parity_test_custom.py > /tmp/parity_test_output.log 2>&1 &
TEST_PID=$!

# Monitor test progress
echo -e "${YELLOW}⏳ Test running (PID: $TEST_PID)...${NC}"
echo -e "${BLUE}💡 Monitor progress: tail -f /tmp/parity_test_output.log${NC}"

# Wait for test completion
wait $TEST_PID
TEST_EXIT_CODE=$?

echo ""
if [ $TEST_EXIT_CODE -eq 0 ] || [ $TEST_EXIT_CODE -eq 124 ]; then
    echo -e "${GREEN}✅ Parity test completed${NC}"
else
    echo -e "${YELLOW}⚠️  Test completed with status: $TEST_EXIT_CODE${NC}"
fi

# Show test output
echo -e "${BLUE}📋 Test Output:${NC}"
tail -20 /tmp/parity_test_output.log

echo ""
echo -e "${PURPLE}📊 RUNNING ANALYSIS${NC}"
echo "==================="

# Run the analysis
python3 gazebo_parity_analysis.py

echo ""
echo -e "${GREEN}🎯 PARITY TEST COMPLETE!${NC}"
echo ""
echo -e "${BLUE}📁 Files Created:${NC}"
echo "   Test Log: /tmp/parity_test_output.log"
echo "   Test Data: /tmp/parity_test_real_*.json"
echo "   Analysis: /tmp/parity_analysis_*.json"

echo ""
echo -e "${PURPLE}🔧 Next Steps:${NC}"
echo "   • Review analysis results above"
echo "   • Tune HSV parameters if grass detection is low"
echo "   • Check camera frame rates if below 25Hz"
echo "   • Test with actual obstacles for obstacle detection"
echo "   • Run extended tests (5+ minutes) for better statistics"

echo ""
echo -e "${GREEN}✅ Parity test runner complete!${NC}"

# Cleanup
rm -f /tmp/run_parity_test_custom.py