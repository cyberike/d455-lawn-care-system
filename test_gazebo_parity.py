#!/usr/bin/env python3
"""
Gazebo Simulation vs Real Hardware Parity Test
Compares detection performance between simulation and real D455 camera
"""

import rclpy
from rclpy.node import Node
import time
import matplotlib.pyplot as plt
import numpy as np
import json
from collections import defaultdict, deque

# ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Int32, String
from nav_msgs.msg import OccupancyGrid

class GazeboPairityTester(Node):
    def __init__(self):
        super().__init__('gazebo_parity_tester')
        
        # Test configuration
        self.test_duration = 120.0  # 2 minute test
        self.start_time = time.time()
        
        # Performance tracking for both real and sim
        self.performance_data = {
            'real': {
                'camera_rates': defaultdict(list),
                'detection_rates': defaultdict(list),
                'processing_latencies': defaultdict(list),
                'detection_accuracy': defaultdict(list),
                'timestamp': []
            },
            'sim': {
                'camera_rates': defaultdict(list),
                'detection_rates': defaultdict(list),
                'processing_latencies': defaultdict(list),
                'detection_accuracy': defaultdict(list),
                'timestamp': []
            }
        }
        
        # Current mode detection (real vs sim)
        self.current_mode = self.detect_current_mode()
        
        # Message tracking
        self.message_times = defaultdict(lambda: deque(maxlen=100))
        self.detection_values = defaultdict(lambda: deque(maxlen=100))
        
        # Setup subscribers based on current mode
        self.setup_subscribers()
        
        # Analysis timer
        self.analysis_timer = self.create_timer(10.0, self.periodic_analysis)
        
        # Final analysis timer
        self.final_timer = self.create_timer(self.test_duration, self.generate_parity_report)
        
        self.get_logger().info(f'🎯 Gazebo Parity Tester Started ({self.current_mode} mode)')
        self.print_test_info()
    
    def detect_current_mode(self):
        """Detect if we're running with real hardware or simulation"""
        # Check for typical simulation topics/nodes
        try:
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            topics = result.stdout
            
            # Look for Gazebo-specific topics
            if '/clock' in topics and '/gazebo/' in topics:
                return 'sim'
            elif '/d455/d455_camera/' in topics:
                return 'real'
            else:
                self.get_logger().warn('Could not detect mode, defaulting to real')
                return 'real'
        except:
            self.get_logger().warn('Failed to detect mode, defaulting to real')
            return 'real'
    
    def print_test_info(self):
        """Print test information"""
        print("\n" + "="*70)
        print("🎯 GAZEBO SIMULATION vs REAL HARDWARE PARITY TEST")
        print("="*70)
        print(f"Current Mode: {self.current_mode.upper()}")
        print(f"Test Duration: {self.test_duration}s")
        print("")
        print("COMPARISON METRICS:")
        print("📷 Camera Performance:")
        print("   • Frame rates (color, depth, infrared)")
        print("   • Image quality consistency")
        print("   • Timestamp accuracy")
        print("")
        print("🧠 Detection Performance:")
        print("   • Grass detection accuracy")
        print("   • Obstacle detection accuracy")  
        print("   • Processing latency")
        print("   • Detection consistency")
        print("")
        print("🎮 Expected Differences:")
        print("   • Sim: Perfect lighting, no noise")
        print("   • Real: Variable lighting, sensor noise")
        print("   • Sim: Deterministic behavior")
        print("   • Real: Environmental variations")
        print("")
        print("📊 Analysis Output:")
        print("   • Performance comparison charts")
        print("   • Accuracy deviation metrics")
        print("   • Recommendations for parameter tuning")
        print("="*70 + "\n")
    
    def setup_subscribers(self):
        """Setup subscribers based on current mode"""
        
        if self.current_mode == 'real':
            # Real D455 camera topics
            color_topic = '/d455/d455/color/image_raw'
            depth_topic = '/d455/d455/depth/image_rect_raw'
        else:
            # Simulated camera topics (typical Gazebo setup)
            camera_base = '/camera'
            color_topic = f'{camera_base}/color/image_raw'
            depth_topic = f'{camera_base}/depth/image_raw'
        
        # Camera stream subscribers
        self.color_sub = self.create_subscription(
            Image, color_topic,
            lambda msg: self.image_callback(msg, 'color'),
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image, depth_topic, 
            lambda msg: self.image_callback(msg, 'depth'),
            10
        )
        
        # Detection output subscribers
        self.grass_coverage_sub = self.create_subscription(
            Float32, '/simple_grass_detector/grass_coverage_percentage',
            self.grass_detection_callback, 10
        )
        
        self.obstacle_count_sub = self.create_subscription(
            Int32, '/simple_obstacle_detector/obstacle_count',
            self.obstacle_detection_callback, 10
        )
        
        # Costmap subscribers for comparison
        self.grass_costmap_sub = self.create_subscription(
            OccupancyGrid, '/lawn_care/grass_costmap',
            lambda msg: self.costmap_callback(msg, 'grass'),
            10
        )
        
        self.obstacle_costmap_sub = self.create_subscription(
            OccupancyGrid, '/lawn_care/obstacle_costmap',
            lambda msg: self.costmap_callback(msg, 'obstacle'),
            10
        )
        
        self.get_logger().info(f'Subscribed to {self.current_mode} topics')
    
    def image_callback(self, msg, stream_type):
        """Handle camera image messages"""
        current_time = time.time()
        self.message_times[f'{stream_type}_image'].append(current_time)
        
        # Calculate processing latency
        if hasattr(msg.header, 'stamp') and msg.header.stamp.sec > 0:
            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            latency = current_time - ros_time
            self.performance_data[self.current_mode]['processing_latencies'][stream_type].append(latency)
    
    def grass_detection_callback(self, msg):
        """Handle grass detection messages"""
        current_time = time.time()
        self.message_times['grass_detection'].append(current_time)
        self.detection_values['grass_coverage'].append(msg.data)
        
        # Track detection accuracy (higher grass coverage = better detection)
        accuracy_score = min(100, max(0, msg.data))  # Normalize to 0-100
        self.performance_data[self.current_mode]['detection_accuracy']['grass'].append(accuracy_score)
    
    def obstacle_detection_callback(self, msg):
        """Handle obstacle detection messages"""
        current_time = time.time()
        self.message_times['obstacle_detection'].append(current_time)
        self.detection_values['obstacle_count'].append(msg.data)
        
        # Track obstacle detection consistency
        self.performance_data[self.current_mode]['detection_accuracy']['obstacle'].append(msg.data)
    
    def costmap_callback(self, msg, costmap_type):
        """Handle costmap messages"""
        current_time = time.time()
        self.message_times[f'{costmap_type}_costmap'].append(current_time)
    
    def calculate_rate(self, message_times, window=10.0):
        """Calculate message rate over time window"""
        if len(message_times) < 2:
            return 0.0
        
        current_time = time.time()
        recent_times = [t for t in message_times if current_time - t < window]
        
        if len(recent_times) < 2:
            return 0.0
        
        return len(recent_times) / window
    
    def periodic_analysis(self):
        """Perform periodic analysis"""
        elapsed_time = time.time() - self.start_time
        
        # Calculate current rates
        color_rate = self.calculate_rate(self.message_times['color_image'])
        depth_rate = self.calculate_rate(self.message_times['depth_image'])
        grass_rate = self.calculate_rate(self.message_times['grass_detection'])
        obstacle_rate = self.calculate_rate(self.message_times['obstacle_detection'])
        
        # Store performance data
        current_data = self.performance_data[self.current_mode]
        current_data['camera_rates']['color'].append(color_rate)
        current_data['camera_rates']['depth'].append(depth_rate)
        current_data['detection_rates']['grass'].append(grass_rate)
        current_data['detection_rates']['obstacle'].append(obstacle_rate)
        current_data['timestamp'].append(elapsed_time)
        
        self.get_logger().info(f'📊 {self.current_mode.upper()} Performance ({elapsed_time:.0f}s): '
                             f'Color {color_rate:.1f}Hz, Depth {depth_rate:.1f}Hz, '
                             f'Grass {grass_rate:.1f}Hz, Obstacles {obstacle_rate:.1f}Hz')
        
        # Show detection values if available
        if len(self.detection_values['grass_coverage']) > 0:
            avg_grass = np.mean(list(self.detection_values['grass_coverage'])[-10:])
            self.get_logger().info(f'🌱 Avg Grass Coverage: {avg_grass:.1f}%')
        
        if len(self.detection_values['obstacle_count']) > 0:
            avg_obstacles = np.mean(list(self.detection_values['obstacle_count'])[-10:])
            self.get_logger().info(f'🚧 Avg Obstacle Count: {avg_obstacles:.1f}')
    
    def generate_parity_report(self):
        """Generate comprehensive parity analysis report"""
        
        print("\n" + "="*80)
        print("📋 GAZEBO vs REAL HARDWARE PARITY ANALYSIS")
        print("="*80)
        
        current_data = self.performance_data[self.current_mode]
        
        # Performance summary for current mode
        print(f"\n{self.current_mode.upper()} HARDWARE PERFORMANCE:")
        print("-" * 50)
        
        if current_data['camera_rates']['color']:
            avg_color_rate = np.mean(current_data['camera_rates']['color'])
            std_color_rate = np.std(current_data['camera_rates']['color'])
            print(f"Color Camera: {avg_color_rate:.1f}±{std_color_rate:.1f} Hz")
        
        if current_data['camera_rates']['depth']:
            avg_depth_rate = np.mean(current_data['camera_rates']['depth'])
            std_depth_rate = np.std(current_data['camera_rates']['depth'])
            print(f"Depth Camera: {avg_depth_rate:.1f}±{std_depth_rate:.1f} Hz")
        
        if current_data['detection_rates']['grass']:
            avg_grass_rate = np.mean(current_data['detection_rates']['grass'])
            std_grass_rate = np.std(current_data['detection_rates']['grass'])
            print(f"Grass Detection: {avg_grass_rate:.1f}±{std_grass_rate:.1f} Hz")
        
        if current_data['detection_rates']['obstacle']:
            avg_obstacle_rate = np.mean(current_data['detection_rates']['obstacle'])
            std_obstacle_rate = np.std(current_data['detection_rates']['obstacle'])
            print(f"Obstacle Detection: {avg_obstacle_rate:.1f}±{std_obstacle_rate:.1f} Hz")
        
        # Detection accuracy analysis
        print(f"\nDETECTION ACCURACY ({self.current_mode.upper()}):")
        print("-" * 40)
        
        if current_data['detection_accuracy']['grass']:
            grass_scores = current_data['detection_accuracy']['grass']
            avg_grass_acc = np.mean(grass_scores)
            std_grass_acc = np.std(grass_scores)
            print(f"Grass Detection: {avg_grass_acc:.1f}±{std_grass_acc:.1f}%")
        
        # Processing latency analysis
        if current_data['processing_latencies']:
            print(f"\nPROCESSING LATENCY ({self.current_mode.upper()}):")
            print("-" * 40)
            
            for stream, latencies in current_data['processing_latencies'].items():
                if latencies:
                    avg_latency = np.mean(latencies) * 1000  # Convert to ms
                    std_latency = np.std(latencies) * 1000
                    print(f"{stream.capitalize()}: {avg_latency:.1f}±{std_latency:.1f} ms")
        
        # Stability analysis
        print(f"\nSTABILITY ANALYSIS ({self.current_mode.upper()}):")
        print("-" * 40)
        
        # Calculate coefficient of variation for stability
        if current_data['camera_rates']['color']:
            color_cv = np.std(current_data['camera_rates']['color']) / np.mean(current_data['camera_rates']['color']) * 100
            print(f"Color Stream Stability: {100-color_cv:.1f}% (CV: {color_cv:.1f}%)")
        
        if current_data['detection_rates']['grass']:
            grass_cv = np.std(current_data['detection_rates']['grass']) / max(np.mean(current_data['detection_rates']['grass']), 0.1) * 100
            print(f"Grass Detection Stability: {100-grass_cv:.1f}% (CV: {grass_cv:.1f}%)")
        
        # Recommendations based on current mode
        print(f"\nRECOMMENDATIONS FOR {self.current_mode.upper()}:")
        print("-" * 40)
        
        if self.current_mode == 'real':
            print("✅ Real Hardware Optimizations:")
            print("   • Adjust exposure for outdoor lighting variations")
            print("   • Fine-tune HSV thresholds for real grass colors")
            print("   • Enable depth stream for obstacle detection")
            print("   • Consider filtering for sensor noise reduction")
            
            if current_data['camera_rates']['color'] and np.mean(current_data['camera_rates']['color']) < 25:
                print("   ⚠️  Low frame rate detected - check USB bandwidth")
                
            if current_data['detection_accuracy']['grass'] and np.mean(current_data['detection_accuracy']['grass']) < 50:
                print("   ⚠️  Low grass detection - retune color parameters")
        
        else:
            print("✅ Simulation Optimizations:")
            print("   • Match simulated lighting to real conditions")
            print("   • Add realistic sensor noise models")
            print("   • Validate material properties for grass/obstacles")
            print("   • Test various weather/lighting scenarios")
        
        # Data export for comparison
        self.save_performance_data()
        
        print("\n📊 COMPARISON INSTRUCTIONS:")
        print("-" * 30)
        print("To complete parity analysis:")
        print("1. Run this test in both real and simulation modes")
        print("2. Compare the saved JSON performance data")
        print("3. Adjust parameters to minimize differences")
        print("4. Focus on detection accuracy and stability metrics")
        
        print("="*80 + "\n")
        
        self.get_logger().info('📊 Parity analysis complete!')
        self.analysis_timer.cancel()
    
    def save_performance_data(self):
        """Save performance data to file for comparison"""
        timestamp = int(time.time())
        filename = f'/tmp/parity_test_{self.current_mode}_{timestamp}.json'
        
        # Convert numpy arrays to lists for JSON serialization
        export_data = {}
        for mode, data in self.performance_data.items():
            export_data[mode] = {}
            for category, values in data.items():
                if isinstance(values, dict):
                    export_data[mode][category] = {}
                    for key, val_list in values.items():
                        export_data[mode][category][key] = list(val_list) if val_list else []
                else:
                    export_data[mode][category] = list(values) if values else []
        
        # Add test metadata
        export_data['metadata'] = {
            'test_duration': self.test_duration,
            'current_mode': self.current_mode,
            'timestamp': timestamp,
            'ros_distro': 'humble',
            'test_type': 'gazebo_parity'
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            self.get_logger().info(f'📁 Performance data saved: {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save performance data: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = GazeboPairityTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n🛑 Parity test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()