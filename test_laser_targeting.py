#!/usr/bin/env python3
"""
Test script for laser weed incisor targeting functionality
"""

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool, String

class LaserTargetingTest(Node):
    def __init__(self):
        super().__init__('laser_targeting_test')
        
        # Publishers for controlling laser
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/laser_weed_incisor_simple/emergency_stop', 10
        )
        self.safety_enable_pub = self.create_publisher(
            Bool, '/laser_weed_incisor_simple/safety_enable', 10
        )
        
        # Subscriber for status monitoring
        self.status_sub = self.create_subscription(
            String, '/laser_weed_incisor_simple/laser_status', 
            self.status_callback, 10
        )
        
        self.latest_status = None
        self.get_logger().info('Laser Targeting Test Node initialized')
        
    def status_callback(self, msg):
        """Monitor laser status"""
        self.latest_status = msg.data
        
    def run_targeting_test(self):
        """Run comprehensive targeting test sequence"""
        self.get_logger().info('Starting laser targeting test sequence...')
        
        # Test 1: Enable safety system
        self.get_logger().info('Test 1: Enabling safety system')
        safety_msg = Bool()
        safety_msg.data = True
        self.safety_enable_pub.publish(safety_msg)
        time.sleep(2)
        
        if self.latest_status:
            self.get_logger().info(f'Status after safety enable: {self.latest_status}')
            
        # Test 2: Simulate weed detection trigger
        # (The simple version runs its own targeting simulation)
        self.get_logger().info('Test 2: Monitoring autonomous targeting (simulated)')
        time.sleep(3)
        
        # Test 3: Test emergency stop during operation
        self.get_logger().info('Test 3: Testing emergency stop')
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        time.sleep(2)
        
        if self.latest_status:
            self.get_logger().info(f'Status after emergency stop: {self.latest_status}')
            
        # Test 4: Recovery from emergency stop
        self.get_logger().info('Test 4: Testing recovery from emergency stop')
        stop_msg.data = False
        self.emergency_stop_pub.publish(stop_msg)
        time.sleep(1)
        
        # Re-enable safety
        safety_msg.data = True
        self.safety_enable_pub.publish(safety_msg)
        time.sleep(2)
        
        if self.latest_status:
            self.get_logger().info(f'Final status: {self.latest_status}')
            
        # Test 5: Disable system safely
        self.get_logger().info('Test 5: Safely disabling laser system')
        safety_msg.data = False
        self.safety_enable_pub.publish(safety_msg)
        time.sleep(2)
        
        if self.latest_status:
            self.get_logger().info(f'Status after disable: {self.latest_status}')
            
        self.get_logger().info('Laser targeting test sequence completed!')
        return True

def main():
    rclpy.init()
    
    test_node = LaserTargetingTest()
    
    # Give system time to initialize
    time.sleep(2)
    
    # Run the test sequence
    test_node.run_targeting_test()
    
    # Keep node alive briefly to receive final status updates
    for i in range(5):
        rclpy.spin_once(test_node, timeout_sec=1.0)
        
    test_node.destroy_node()
    rclpy.shutdown()
    print("Test completed successfully!")

if __name__ == '__main__':
    main()