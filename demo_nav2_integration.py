#!/usr/bin/env python3
"""
D455 Lawn Care Nav2 Integration Demo
Demonstrates the complete pipeline from detection to navigation costmaps
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np

# ROS 2 message types
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

class Nav2IntegrationDemo(Node):
    def __init__(self):
        super().__init__('nav2_integration_demo')
        
        # Track received messages
        self.received_grass_costmap = False
        self.received_obstacle_costmap = False  
        self.received_combined_costmap = False
        self.received_coverage_path = False
        
        # Test data simulation
        self.test_scenario = 0
        self.scenarios = [
            {'grass': 45.0, 'obstacles': 0, 'description': 'High grass, no obstacles'},
            {'grass': 15.0, 'obstacles': 3, 'description': 'Low grass, multiple obstacles'},
            {'grass': 80.0, 'obstacles': 1, 'description': 'Very high grass, single obstacle'},
            {'grass': 5.0, 'obstacles': 0, 'description': 'Minimal grass coverage'},
        ]
        
        # Setup communication
        self.setup_publishers()
        self.setup_subscribers()
        
        # Demo timer
        self.demo_timer = self.create_timer(5.0, self.run_demo_scenario)
        
        self.get_logger().info('🚀 D455 Lawn Care Nav2 Integration Demo Started')
        self.print_system_overview()
    
    def setup_publishers(self):
        """Setup publishers for detection simulation"""
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
    
    def setup_subscribers(self):
        """Setup subscribers to monitor Nav2 integration"""
        
        # Costmap subscribers
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
        
        # Coverage path subscriber
        self.coverage_path_sub = self.create_subscription(
            Path,
            '/lawn_care/coverage_path',
            self.coverage_path_callback,
            10
        )
        
        # Visualization markers
        self.markers_sub = self.create_subscription(
            MarkerArray,
            '/lawn_care/costmap_markers',
            self.markers_callback,
            10
        )
    
    def print_system_overview(self):
        """Print system architecture overview"""
        print("\n" + "="*70)
        print("🌱 D455 LAWN CARE NAV2 INTEGRATION ARCHITECTURE")
        print("="*70)
        print("📡 DETECTION LAYER:")
        print("   │ D455 Camera → Grass Detector → Obstacle Detector")
        print("   │")
        print("🗺️  COSTMAP LAYER:")
        print("   │ Detection Data → Costmap Generator → Nav2 Costmaps")
        print("   │ • Grass Coverage Costmap (lower cost = more grass)")
        print("   │ • Obstacle Costmap (higher cost = obstacles)")
        print("   │ • Combined Costmap (Nav2 compatible)")
        print("   │")
        print("🛤️  PLANNING LAYER:")
        print("   │ Coverage Path Planner → Boustrophedon Patterns")
        print("   │ Nav2 Navigation Stack → Autonomous Execution")
        print("   │")
        print("🎯 INTEGRATION TOPICS:")
        print("   │ /lawn_care/grass_coverage_costmap")
        print("   │ /lawn_care/obstacle_costmap")
        print("   │ /lawn_care/combined_costmap")
        print("   │ /lawn_care/coverage_path")
        print("="*70 + "\n")
    
    def run_demo_scenario(self):
        """Run demonstration scenarios"""
        
        if self.test_scenario >= len(self.scenarios):
            self.get_logger().info('🏁 All demo scenarios completed!')
            self.print_final_status()
            return
        
        scenario = self.scenarios[self.test_scenario]
        
        self.get_logger().info(f"📋 Scenario {self.test_scenario + 1}/{len(self.scenarios)}: {scenario['description']}")
        
        # Publish test data
        grass_msg = Float32()
        grass_msg.data = scenario['grass']
        self.grass_coverage_pub.publish(grass_msg)
        
        obstacle_msg = Int32()  
        obstacle_msg.data = scenario['obstacles']
        self.obstacle_count_pub.publish(obstacle_msg)
        
        self.get_logger().info(f"📤 Published: {scenario['grass']:.1f}% grass, {scenario['obstacles']} obstacles")
        
        self.test_scenario += 1
    
    def grass_costmap_callback(self, msg):
        """Handle grass coverage costmap"""
        if not self.received_grass_costmap:
            self.received_grass_costmap = True
            self.get_logger().info(f'✅ Grass Coverage Costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.2f}m/px')
            self.analyze_costmap(msg, "Grass Coverage")
    
    def obstacle_costmap_callback(self, msg):
        """Handle obstacle costmap"""
        if not self.received_obstacle_costmap:
            self.received_obstacle_costmap = True
            self.get_logger().info(f'✅ Obstacle Costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.2f}m/px')
            self.analyze_costmap(msg, "Obstacle")
    
    def combined_costmap_callback(self, msg):
        """Handle combined Nav2 costmap"""
        if not self.received_combined_costmap:
            self.received_combined_costmap = True
            self.get_logger().info(f'✅ Combined Nav2 Costmap: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.2f}m/px')
            self.analyze_costmap(msg, "Combined Nav2")
    
    def coverage_path_callback(self, msg):
        """Handle coverage path"""
        if not self.received_coverage_path:
            self.received_coverage_path = True
            self.get_logger().info(f'✅ Coverage Path: {len(msg.poses)} waypoints')
            
            if len(msg.poses) > 0:
                start_pose = msg.poses[0]
                end_pose = msg.poses[-1]
                self.get_logger().info(f'   Path from ({start_pose.pose.position.x:.1f}, {start_pose.pose.position.y:.1f}) '
                                     f'to ({end_pose.pose.position.x:.1f}, {end_pose.pose.position.y:.1f})')
    
    def markers_callback(self, msg):
        """Handle visualization markers"""
        if len(msg.markers) > 0:
            self.get_logger().debug(f'Received {len(msg.markers)} visualization markers')
    
    def analyze_costmap(self, msg, costmap_type):
        """Analyze costmap data"""
        try:
            costmap_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            
            # Count different cost levels
            unknown = np.sum(costmap_data == -1)
            free = np.sum((costmap_data >= 0) & (costmap_data < 50))
            moderate = np.sum((costmap_data >= 50) & (costmap_data < 100))
            high = np.sum(costmap_data >= 100)
            
            total = msg.info.width * msg.info.height
            
            self.get_logger().info(f'   {costmap_type} Analysis: '
                                 f'Free: {100*free/total:.1f}%, '
                                 f'Moderate: {100*moderate/total:.1f}%, '
                                 f'High: {100*high/total:.1f}%, '
                                 f'Unknown: {100*unknown/total:.1f}%')
            
        except Exception as e:
            self.get_logger().debug(f'Error analyzing {costmap_type} costmap: {e}')
    
    def print_final_status(self):
        """Print final integration status"""
        print("\n" + "="*50)
        print("🔍 INTEGRATION TEST RESULTS")
        print("="*50)
        
        status_items = [
            ("Grass Coverage Costmap", self.received_grass_costmap),
            ("Obstacle Costmap", self.received_obstacle_costmap),
            ("Combined Nav2 Costmap", self.received_combined_costmap),
            ("Coverage Path Planning", self.received_coverage_path),
        ]
        
        all_working = True
        for name, status in status_items:
            emoji = "✅" if status else "❌"
            print(f"{emoji} {name}")
            if not status:
                all_working = False
        
        print("="*50)
        
        if all_working:
            print("🎉 INTEGRATION SUCCESSFUL!")
            print("   All D455 detections are successfully feeding into Nav2 costmaps")
            print("   Coverage path planning is generating mowing patterns")
            print("   System ready for autonomous lawn mowing!")
        else:
            print("⚠️  INTEGRATION INCOMPLETE")
            print("   Some components are not responding")
            print("   Check that all nodes are running properly")
        
        print("="*50 + "\n")
        
        # Shutdown after analysis
        self.get_logger().info('Demo completed - shutting down')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = Nav2IntegrationDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        print("\n🛑 Demo interrupted by user")
    except Exception as e:
        print(f"❌ Demo error: {e}")
    finally:
        if 'demo' in locals():
            demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()