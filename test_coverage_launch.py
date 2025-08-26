#!/usr/bin/env python3
"""
Simple Coverage Planner Test Launch
Quick test setup for coverage path generation
"""

import subprocess
import time
import signal
import sys
import os

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print('\n🛑 Stopping coverage planner test...')
    sys.exit(0)

def main():
    print("🌱 D455 Lawn Care Coverage Planner Test")
    print("="*50)
    
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check if we can source the workspace
    workspace_setup = "install/setup.bash"
    if not os.path.exists(workspace_setup):
        print("❌ Workspace not built. Run: colcon build --packages-select d455_lawn_care")
        return
    
    print("Starting components...")
    
    # Start simple coverage planner
    print("1. Starting simple coverage planner...")
    planner_cmd = f"source {workspace_setup} && ros2 run d455_lawn_care simple_coverage_planner"
    planner_process = subprocess.Popen(planner_cmd, shell=True, 
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    time.sleep(2)
    
    # Start test validator  
    print("2. Starting pattern test validator...")
    test_cmd = f"source {workspace_setup} && python3 test_coverage_patterns.py"
    test_process = subprocess.Popen(test_cmd, shell=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    print("\n✅ Test setup complete!")
    print("📋 The test will:")
    print("   • Generate 4 different mowing patterns")
    print("   • Validate path generation")
    print("   • Analyze coverage efficiency")
    print("   • Create visualization plots")
    print("\n⏱️  Test duration: ~2.5 minutes")
    print("🖥️  Monitor with: ros2 topic list | grep lawn_care")
    print("👁️  Visualize with: ros2 run rviz2 rviz2")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        # Wait for processes
        planner_process.wait()
        test_process.wait()
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
        
        # Cleanup processes
        if planner_process.poll() is None:
            planner_process.terminate()
        if test_process.poll() is None:
            test_process.terminate()
            
        time.sleep(1)
        
        # Force kill if needed
        try:
            planner_process.kill()
            test_process.kill()
        except:
            pass
    
    print("🏁 Coverage planner test finished")

if __name__ == '__main__':
    main()