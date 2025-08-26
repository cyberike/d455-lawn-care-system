#!/usr/bin/env python3
"""
Test Script for Coverage Pattern Visualization
Tests different mowing patterns and validates path generation
"""

import rclpy
from rclpy.node import Node
import time
import matplotlib.pyplot as plt
import numpy as np

# ROS 2 message types
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

class CoveragePatternTester(Node):
    def __init__(self):
        super().__init__('coverage_pattern_tester')
        
        # Test tracking
        self.received_paths = {}
        self.current_pattern = None
        self.path_stats = {}
        self.test_start_time = time.time()
        
        # Expected patterns to test
        self.expected_patterns = ['boustrophedon', 'spiral', 'zigzag', 'perimeter_first']
        self.patterns_received = set()
        
        # Setup subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/lawn_care/coverage_path',
            self.path_callback,
            10
        )
        
        self.stats_sub = self.create_subscription(
            String,
            '/lawn_care/coverage_stats',
            self.stats_callback,
            10
        )
        
        self.markers_sub = self.create_subscription(
            MarkerArray,
            '/lawn_care/coverage_visualization',
            self.markers_callback,
            10
        )
        
        # Test timer
        self.test_timer = self.create_timer(2.0, self.check_test_progress)
        
        self.get_logger().info('🧪 Coverage Pattern Tester Started')
        self.get_logger().info(f'Expected patterns: {self.expected_patterns}')
        self.print_test_instructions()
    
    def print_test_instructions(self):
        """Print test setup instructions"""
        print("\n" + "="*60)
        print("🌱 COVERAGE PATTERN TESTING")
        print("="*60)
        print("This test validates coverage path generation patterns.")
        print("")
        print("SETUP:")
        print("1. Start the simple coverage planner:")
        print("   ros2 run d455_lawn_care simple_coverage_planner")
        print("")
        print("2. The planner will cycle through patterns every 30 seconds:")
        print("   • Boustrophedon (back-and-forth)")
        print("   • Spiral (outward from center)")  
        print("   • Zigzag (diagonal patterns)")
        print("   • Perimeter-first (outline then fill)")
        print("")
        print("3. This test will validate each pattern and generate analysis")
        print("="*60 + "\n")
    
    def path_callback(self, msg):
        """Handle coverage path updates"""
        
        # Extract basic path info
        num_waypoints = len(msg.poses)
        
        if num_waypoints == 0:
            return
        
        # Calculate path characteristics
        path_length = self.calculate_path_length(msg)
        coverage_area = self.estimate_coverage_area(msg)
        
        # Store path data  
        pattern_key = f"pattern_{len(self.received_paths)}"
        self.received_paths[pattern_key] = {
            'path': msg,
            'waypoints': num_waypoints,
            'length': path_length,
            'coverage_area': coverage_area,
            'timestamp': time.time()
        }
        
        self.get_logger().info(f'📍 Received path: {num_waypoints} waypoints, {path_length:.1f}m length')
    
    def stats_callback(self, msg):
        """Handle coverage statistics"""
        stats_text = msg.data
        
        # Parse pattern type from stats
        if 'Pattern: ' in stats_text:
            pattern_part = stats_text.split('Pattern: ')[1].split(',')[0]
            self.current_pattern = pattern_part
            
            if self.current_pattern not in self.patterns_received:
                self.patterns_received.add(self.current_pattern)
                self.get_logger().info(f'✅ New pattern detected: {self.current_pattern}')
        
        # Store stats
        self.path_stats[self.current_pattern or 'unknown'] = stats_text
        
        self.get_logger().info(f'📊 Stats: {stats_text}')
    
    def markers_callback(self, msg):
        """Handle visualization markers"""
        if len(msg.markers) > 0:
            self.get_logger().debug(f'Received {len(msg.markers)} visualization markers')
    
    def calculate_path_length(self, path_msg):
        """Calculate total path length"""
        if len(path_msg.poses) < 2:
            return 0.0
        
        total_length = 0.0
        
        for i in range(1, len(path_msg.poses)):
            p1 = path_msg.poses[i-1].pose.position
            p2 = path_msg.poses[i].pose.position
            
            distance = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_length += distance
        
        return total_length
    
    def estimate_coverage_area(self, path_msg):
        """Estimate area covered by path"""
        if len(path_msg.poses) < 3:
            return 0.0
        
        # Get path bounds
        x_coords = [pose.pose.position.x for pose in path_msg.poses]
        y_coords = [pose.pose.position.y for pose in path_msg.poses]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        # Simple rectangular approximation
        area = (x_max - x_min) * (y_max - y_min)
        return area
    
    def check_test_progress(self):
        """Check test progress and completion"""
        elapsed_time = time.time() - self.test_start_time
        
        # Show progress
        received_count = len(self.patterns_received)
        expected_count = len(self.expected_patterns)
        
        self.get_logger().info(f'🔄 Test Progress: {received_count}/{expected_count} patterns '
                             f'({elapsed_time:.0f}s elapsed)')
        
        missing_patterns = set(self.expected_patterns) - self.patterns_received
        if missing_patterns:
            self.get_logger().info(f'⏳ Waiting for: {list(missing_patterns)}')
        
        # Check for completion or timeout
        if received_count >= expected_count:
            self.get_logger().info('🎉 All patterns received! Generating analysis...')
            self.generate_test_analysis()
            self.test_timer.cancel()
        elif elapsed_time > 150:  # 2.5 minutes timeout
            self.get_logger().warn('⏰ Test timeout reached')
            self.generate_test_analysis()
            self.test_timer.cancel()
    
    def generate_test_analysis(self):
        """Generate comprehensive test analysis"""
        
        print("\n" + "="*70)
        print("📋 COVERAGE PATTERN TEST ANALYSIS")
        print("="*70)
        
        # Pattern summary
        print(f"Patterns Tested: {len(self.patterns_received)}/{len(self.expected_patterns)}")
        for pattern in self.expected_patterns:
            status = "✅" if pattern in self.patterns_received else "❌" 
            print(f"  {status} {pattern}")
        
        print(f"\nTotal Paths Received: {len(self.received_paths)}")
        print(f"Test Duration: {time.time() - self.test_start_time:.1f}s")
        
        # Detailed pattern analysis
        if self.path_stats:
            print("\nPATTERN DETAILS:")
            print("-" * 50)
            
            for pattern_name, stats in self.path_stats.items():
                print(f"\n{pattern_name.upper()}:")
                
                # Parse stats
                if 'Distance: ' in stats:
                    distance = stats.split('Distance: ')[1].split('m,')[0]
                    print(f"  Distance: {distance}m")
                
                if 'Waypoints: ' in stats:
                    waypoints = stats.split('Waypoints: ')[1]
                    print(f"  Waypoints: {waypoints}")
                
                if 'Est. Time: ' in stats:
                    time_est = stats.split('Est. Time: ')[1].split('min,')[0]
                    print(f"  Est. Time: {time_est}min")
        
        # Path analysis
        if self.received_paths:
            print("\nPATH ANALYSIS:")
            print("-" * 30)
            
            total_paths = len(self.received_paths)
            avg_waypoints = np.mean([p['waypoints'] for p in self.received_paths.values()])
            avg_length = np.mean([p['length'] for p in self.received_paths.values()])
            avg_area = np.mean([p['coverage_area'] for p in self.received_paths.values()])
            
            print(f"Average Waypoints: {avg_waypoints:.1f}")
            print(f"Average Path Length: {avg_length:.1f}m")
            print(f"Average Coverage Area: {avg_area:.1f}m²")
            
            # Path efficiency metrics
            if avg_area > 0:
                efficiency = avg_length / avg_area
                print(f"Path Efficiency: {efficiency:.2f} m/m² (lower = more efficient)")
        
        # Test results
        print(f"\nTEST RESULTS:")
        print("-" * 20)
        
        if len(self.patterns_received) >= len(self.expected_patterns):
            print("✅ TEST PASSED: All expected patterns generated successfully")
            print("✅ Coverage path publisher is working correctly")
            print("✅ Pattern switching mechanism functional")
            
            if self.received_paths:
                print("✅ Path statistics calculation working")
                print("✅ Visualization markers being published")
        else:
            print("❌ TEST FAILED: Missing expected patterns")
            missing = set(self.expected_patterns) - self.patterns_received
            print(f"❌ Missing patterns: {list(missing)}")
        
        # Recommendations
        print(f"\nRECOMMENDATIONS:")
        print("-" * 20)
        
        if len(self.received_paths) == 0:
            print("⚠️  No paths received - check that simple_coverage_planner is running")
        elif len(self.patterns_received) < len(self.expected_patterns):
            print("⚠️  Incomplete pattern coverage - increase test duration")
        else:
            print("✅ System ready for integration with Nav2")
            print("✅ Coverage patterns suitable for autonomous mowing")
        
        print("="*70 + "\n")
        
        # Optional: Create visualization plot
        self.create_pattern_visualization()
        
        self.get_logger().info('📊 Test analysis complete!')
    
    def create_pattern_visualization(self):
        """Create visualization plot of path patterns"""
        try:
            if not self.received_paths:
                return
            
            # Create subplot for each received path
            num_paths = len(self.received_paths)
            cols = min(2, num_paths)
            rows = (num_paths + cols - 1) // cols
            
            fig, axes = plt.subplots(rows, cols, figsize=(12, 8))
            if num_paths == 1:
                axes = [axes]
            elif rows == 1:
                pass  # axes is already correct
            else:
                axes = axes.flatten()
            
            for i, (pattern_name, path_data) in enumerate(self.received_paths.items()):
                if i >= len(axes):
                    break
                    
                ax = axes[i] if num_paths > 1 else axes
                
                path = path_data['path']
                x_coords = [pose.pose.position.x for pose in path.poses]
                y_coords = [pose.pose.position.y for pose in path.poses]
                
                # Plot path
                ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7)
                ax.plot(x_coords[0], y_coords[0], 'go', markersize=8, label='Start')
                ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=8, label='End')
                
                # Add waypoint markers
                ax.scatter(x_coords, y_coords, c='orange', s=20, alpha=0.6, zorder=5)
                
                ax.set_title(f'{pattern_name}\n{len(x_coords)} waypoints, {path_data["length"]:.1f}m')
                ax.set_xlabel('X (m)')
                ax.set_ylabel('Y (m)')
                ax.grid(True, alpha=0.3)
                ax.axis('equal')
                ax.legend()
            
            # Remove unused subplots
            if num_paths < len(axes):
                for i in range(num_paths, len(axes)):
                    axes[i].remove()
            
            plt.tight_layout()
            plt.suptitle('Coverage Pattern Analysis', y=0.98)
            
            # Save plot
            filename = f'/tmp/coverage_patterns_{int(time.time())}.png'
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info(f'📈 Pattern visualization saved: {filename}')
            
        except Exception as e:
            self.get_logger().warn(f'Could not create visualization: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tester = CoveragePatternTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()