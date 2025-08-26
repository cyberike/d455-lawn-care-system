#!/usr/bin/env python3
"""
Test Script for D455 Costmap Integration
Verifies costmap generation from detection data
"""

import rclpy
from rclpy.node import Node
import time

# ROS 2 message types
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Int32

class CostmapTester(Node):
    def __init__(self):
        super().__init__('costmap_tester')
        
        # Test data
        self.test_grass_coverage = 25.0  # 25% grass coverage
        self.test_obstacle_count = 2     # 2 obstacles detected
        
        # Subscribers to verify costmap generation
        self.grass_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lawn_care/grass_coverage_costmap',
            self.grass_costmap_callback,
            10
        )
        
        self.obstacle_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lawn_care/obstacle_costmap',
            self.obstacle_costmap_callback,
            10
        )
        
        self.combined_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lawn_care/combined_costmap',
            self.combined_costmap_callback,
            10
        )
        
        # Publishers to simulate detection data
        self.grass_coverage_pub = self.create_publisher(
            Float32,
            '/simple_grass_detector/grass_coverage_percentage',
            10
        )
        
        self.obstacle_count_pub = self.create_publisher(
            Int32,
            '/simple_obstacle_detector/obstacle_count',
            10
        )
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.publish_test_data)
        
        # Results tracking
        self.received_grass_costmap = False
        self.received_obstacle_costmap = False
        self.received_combined_costmap = False
        
        self.get_logger().info('Costmap Integration Tester started')
        self.get_logger().info('Publishing test detection data...')
    
    def publish_test_data(self):
        """Publish simulated detection data"""
        
        # Publish grass coverage
        grass_msg = Float32()
        grass_msg.data = self.test_grass_coverage
        self.grass_coverage_pub.publish(grass_msg)
        
        # Publish obstacle count
        obstacle_msg = Int32()
        obstacle_msg.data = self.test_obstacle_count
        self.obstacle_count_pub.publish(obstacle_msg)
        
        self.get_logger().info(f'Published: {self.test_grass_coverage}% grass, {self.test_obstacle_count} obstacles')
        
        # Vary the data slightly for realistic simulation
        self.test_grass_coverage = 15.0 + (time.time() % 10) * 2  # 15-35% range
        self.test_obstacle_count = int(time.time() % 4)  # 0-3 obstacles
    
    def grass_costmap_callback(self, msg):
        """Handle grass coverage costmap"""
        if not self.received_grass_costmap:
            self.received_grass_costmap = True
            self.get_logger().info(f'✅ Received grass coverage costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/px')
            self.check_completion()
    
    def obstacle_costmap_callback(self, msg):
        """Handle obstacle costmap"""
        if not self.received_obstacle_costmap:
            self.received_obstacle_costmap = True
            self.get_logger().info(f'✅ Received obstacle costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/px')
            self.check_completion()
    
    def combined_costmap_callback(self, msg):
        """Handle combined costmap"""
        if not self.received_combined_costmap:
            self.received_combined_costmap = True
            self.get_logger().info(f'✅ Received combined costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/px')
            
            # Analyze costmap data
            import numpy as np
            costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            
            unique_values = np.unique(costmap_data)
            self.get_logger().info(f'Costmap value range: {unique_values}')
            
            # Count different cost levels
            unknown = np.sum(costmap_data == -1)
            free = np.sum((costmap_data >= 0) & (costmap_data < 25))
            low_cost = np.sum((costmap_data >= 25) & (costmap_data < 75))
            high_cost = np.sum((costmap_data >= 75) & (costmap_data < 200))
            lethal = np.sum(costmap_data >= 200)
            
            total_cells = msg.info.width * msg.info.height
            
            self.get_logger().info('Costmap Analysis:')
            self.get_logger().info(f'  Unknown: {unknown}/{total_cells} ({100*unknown/total_cells:.1f}%)')
            self.get_logger().info(f'  Free: {free}/{total_cells} ({100*free/total_cells:.1f}%)')
            self.get_logger().info(f'  Low cost: {low_cost}/{total_cells} ({100*low_cost/total_cells:.1f}%)')
            self.get_logger().info(f'  High cost: {high_cost}/{total_cells} ({100*high_cost/total_cells:.1f}%)')
            self.get_logger().info(f'  Lethal: {lethal}/{total_cells} ({100*lethal/total_cells:.1f}%)')
            
            self.check_completion()
    
    def check_completion(self):
        """Check if all costmaps have been received"""
        if (self.received_grass_costmap and 
            self.received_obstacle_costmap and 
            self.received_combined_costmap):
            
            self.get_logger().info('🎉 All costmaps received successfully!')
            self.get_logger().info('Costmap integration test PASSED')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = CostmapTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()