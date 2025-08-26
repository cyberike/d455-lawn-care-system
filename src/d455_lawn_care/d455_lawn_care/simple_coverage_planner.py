#!/usr/bin/env python3
"""
Simple Lawn Coverage Path Publisher for Testing
Generates basic mowing patterns for Nav2 integration testing
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np

# ROS 2 message types
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
# Nav2 imports (optional - commented out for testing without Nav2)
# from nav2_msgs.action import NavigateThroughPoses  
# from rclpy.action import ActionClient

class SimpleCoveragePlanner(Node):
    def __init__(self):
        super().__init__('simple_coverage_planner')
        
        # Planning parameters
        self.declare_parameter('area_width', 20.0)         # Area width (meters)
        self.declare_parameter('area_height', 15.0)        # Area height (meters)
        self.declare_parameter('area_center_x', 0.0)       # Area center X
        self.declare_parameter('area_center_y', 0.0)       # Area center Y
        self.declare_parameter('mowing_width', 1.2)        # Mower cutting width
        self.declare_parameter('pattern_type', 'boustrophedon')  # Pattern type
        self.declare_parameter('path_spacing', 1.0)        # Distance between paths
        self.declare_parameter('publish_rate', 0.2)        # How often to publish (Hz)
        self.declare_parameter('auto_execute', False)      # Auto execute with Nav2
        
        # Get parameters
        self.area_width = float(self.get_parameter('area_width').value)
        self.area_height = float(self.get_parameter('area_height').value)
        self.area_center_x = float(self.get_parameter('area_center_x').value)
        self.area_center_y = float(self.get_parameter('area_center_y').value)
        self.mowing_width = float(self.get_parameter('mowing_width').value)
        self.pattern_type = self.get_parameter('pattern_type').value
        self.path_spacing = float(self.get_parameter('path_spacing').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.auto_execute = bool(self.get_parameter('auto_execute').value)
        
        # Calculate area bounds
        self.x_min = self.area_center_x - self.area_width / 2
        self.x_max = self.area_center_x + self.area_width / 2
        self.y_min = self.area_center_y - self.area_height / 2  
        self.y_max = self.area_center_y + self.area_height / 2
        
        # Coverage tracking
        self.current_path = None
        self.path_index = 0
        self.available_patterns = ['boustrophedon', 'spiral', 'zigzag', 'perimeter_first']
        
        # Statistics
        self.total_distance = 0.0
        self.coverage_percentage = 0.0
        self.estimated_time = 0.0
        
        # Setup communication
        self.setup_publishers()
        self.setup_subscribers()
        
        # Nav2 action client (if auto execute enabled)
        if self.auto_execute:
            # self.nav_action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
            self.get_logger().warn('Auto execute disabled - Nav2 not available')
        
        # Publishing timer
        self.publish_timer = self.create_timer(1.0/self.publish_rate, self.publish_coverage_path)
        
        # Pattern cycling timer (for demo)
        self.pattern_timer = self.create_timer(30.0, self.cycle_pattern)
        
        self.get_logger().info('Simple Coverage Planner initialized')
        self.get_logger().info(f'Area: {self.area_width}x{self.area_height}m @ ({self.area_center_x}, {self.area_center_y})')
        self.get_logger().info(f'Pattern: {self.pattern_type}, Spacing: {self.path_spacing}m')
        
        # Generate initial path
        self.generate_coverage_path()
    
    def setup_publishers(self):
        """Setup ROS publishers"""
        
        # Main coverage path
        self.path_pub = self.create_publisher(
            Path,
            '/lawn_care/coverage_path',
            10
        )
        
        # Path statistics
        self.stats_pub = self.create_publisher(
            String,
            '/lawn_care/coverage_stats',
            10
        )
        
        # Visualization markers
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/lawn_care/coverage_visualization',
            10
        )
        
        # Area boundary visualization
        self.boundary_pub = self.create_publisher(
            MarkerArray,
            '/lawn_care/mowing_area_boundary',
            10
        )
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        
        # Optional: Listen to costmap for adaptive planning
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lawn_care/combined_costmap',
            self.costmap_callback,
            10
        )
        
        # Grass coverage for prioritization
        self.grass_coverage_sub = self.create_subscription(
            Float32,
            '/simple_grass_detector/grass_coverage_percentage',
            self.grass_coverage_callback,
            10
        )
    
    def costmap_callback(self, msg):
        """Handle costmap updates (future enhancement)"""
        # Could use costmap data to adapt paths in real-time
        pass
    
    def grass_coverage_callback(self, msg):
        """Handle grass coverage updates"""
        # Could use grass coverage to prioritize areas
        pass
    
    def generate_coverage_path(self):
        """Generate coverage path based on selected pattern"""
        
        if self.pattern_type == 'boustrophedon':
            waypoints = self.generate_boustrophedon_pattern()
        elif self.pattern_type == 'spiral':
            waypoints = self.generate_spiral_pattern()
        elif self.pattern_type == 'zigzag':
            waypoints = self.generate_zigzag_pattern()
        elif self.pattern_type == 'perimeter_first':
            waypoints = self.generate_perimeter_first_pattern()
        else:
            self.get_logger().warn(f'Unknown pattern: {self.pattern_type}, using boustrophedon')
            waypoints = self.generate_boustrophedon_pattern()
        
        # Convert to Path message
        self.current_path = self.create_path_message(waypoints)
        
        # Calculate statistics
        self.calculate_path_statistics(waypoints)
        
        self.get_logger().info(f'Generated {self.pattern_type} path with {len(waypoints)} waypoints')
        self.get_logger().info(f'Total distance: {self.total_distance:.1f}m, Est. time: {self.estimated_time:.1f}min')
    
    def generate_boustrophedon_pattern(self):
        """Generate back-and-forth mowing pattern"""
        waypoints = []
        
        # Calculate number of strips
        num_strips = int(self.area_width / self.path_spacing) + 1
        
        for i in range(num_strips):
            # Calculate x position for this strip
            x = self.x_min + i * self.path_spacing
            if x > self.x_max:
                break
                
            if i % 2 == 0:
                # Forward direction (bottom to top)
                waypoints.append((x, self.y_min, 0.0))    # Start
                waypoints.append((x, self.y_max, 0.0))    # End
            else:
                # Reverse direction (top to bottom) 
                waypoints.append((x, self.y_max, math.pi)) # Start (facing opposite)
                waypoints.append((x, self.y_min, math.pi)) # End
        
        return waypoints
    
    def generate_spiral_pattern(self):
        """Generate spiral mowing pattern (outward from center)"""
        waypoints = []
        
        # Start from center and spiral outward
        margin = self.path_spacing
        current_x_min = self.area_center_x - margin
        current_x_max = self.area_center_x + margin
        current_y_min = self.area_center_y - margin
        current_y_max = self.area_center_y + margin
        
        while (current_x_max - current_x_min < self.area_width and 
               current_y_max - current_y_min < self.area_height):
            
            # Ensure we don't exceed area bounds
            current_x_min = max(current_x_min, self.x_min)
            current_x_max = min(current_x_max, self.x_max)
            current_y_min = max(current_y_min, self.y_min)
            current_y_max = min(current_y_max, self.y_max)
            
            # Bottom edge (left to right)
            waypoints.append((current_x_min, current_y_min, 0.0))
            waypoints.append((current_x_max, current_y_min, 0.0))
            
            # Right edge (bottom to top)
            waypoints.append((current_x_max, current_y_max, math.pi/2))
            
            # Top edge (right to left)
            waypoints.append((current_x_min, current_y_max, math.pi))
            
            # Left edge (top to bottom, partial)
            if current_y_max - current_y_min > self.path_spacing:
                waypoints.append((current_x_min, current_y_min + margin, -math.pi/2))
            
            # Expand spiral
            current_x_min -= margin
            current_x_max += margin
            current_y_min -= margin
            current_y_max += margin
        
        return waypoints
    
    def generate_zigzag_pattern(self):
        """Generate diagonal zigzag pattern"""
        waypoints = []
        
        # Calculate diagonal strips
        diagonal_length = math.sqrt(self.area_width**2 + self.area_height**2)
        num_strips = int(diagonal_length / self.path_spacing) + 1
        
        # Angle for diagonal movement
        angle = math.atan2(self.area_height, self.area_width)
        
        for i in range(num_strips):
            # Calculate perpendicular offset
            offset = (i - num_strips/2) * self.path_spacing
            
            # Calculate start and end points for diagonal
            if i % 2 == 0:
                start_x = self.x_min - offset * math.sin(angle)
                start_y = self.y_min + offset * math.cos(angle)
                end_x = self.x_max - offset * math.sin(angle)
                end_y = self.y_max + offset * math.cos(angle)
            else:
                start_x = self.x_max - offset * math.sin(angle)
                start_y = self.y_max + offset * math.cos(angle)
                end_x = self.x_min - offset * math.sin(angle)
                end_y = self.y_min + offset * math.cos(angle)
            
            # Clamp to area bounds
            start_x = max(self.x_min, min(self.x_max, start_x))
            start_y = max(self.y_min, min(self.y_max, start_y))
            end_x = max(self.x_min, min(self.x_max, end_x))
            end_y = max(self.y_min, min(self.y_max, end_y))
            
            if abs(start_x - end_x) > 0.1 or abs(start_y - end_y) > 0.1:
                direction = math.atan2(end_y - start_y, end_x - start_x)
                waypoints.append((start_x, start_y, direction))
                waypoints.append((end_x, end_y, direction))
        
        return waypoints
    
    def generate_perimeter_first_pattern(self):
        """Generate perimeter-first pattern (outline then fill)"""
        waypoints = []
        
        # First, trace the perimeter
        perimeter_margin = self.path_spacing / 2
        
        # Perimeter rectangle
        px_min = self.x_min + perimeter_margin
        px_max = self.x_max - perimeter_margin  
        py_min = self.y_min + perimeter_margin
        py_max = self.y_max - perimeter_margin
        
        # Perimeter waypoints (clockwise)
        waypoints.extend([
            (px_min, py_min, 0.0),      # Bottom left
            (px_max, py_min, 0.0),      # Bottom right
            (px_max, py_max, math.pi/2), # Top right  
            (px_min, py_max, math.pi),   # Top left
            (px_min, py_min, -math.pi/2) # Back to start
        ])
        
        # Then fill interior with boustrophedon
        interior_waypoints = []
        interior_width = px_max - px_min - self.path_spacing
        interior_strips = int(interior_width / self.path_spacing)
        
        for i in range(interior_strips):
            x = px_min + self.path_spacing + i * self.path_spacing
            if x >= px_max - self.path_spacing:
                break
                
            if i % 2 == 0:
                interior_waypoints.extend([
                    (x, py_min + self.path_spacing, 0.0),
                    (x, py_max - self.path_spacing, 0.0)
                ])
            else:
                interior_waypoints.extend([
                    (x, py_max - self.path_spacing, math.pi),
                    (x, py_min + self.path_spacing, math.pi)
                ])
        
        waypoints.extend(interior_waypoints)
        
        return waypoints
    
    def create_path_message(self, waypoints):
        """Convert waypoints to ROS Path message"""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0  
            pose.pose.orientation.z = float(sy)
            pose.pose.orientation.w = float(cy)
            
            path.poses.append(pose)
        
        return path
    
    def calculate_path_statistics(self, waypoints):
        """Calculate path statistics"""
        if len(waypoints) < 2:
            return
            
        self.total_distance = 0.0
        
        for i in range(1, len(waypoints)):
            x1, y1, _ = waypoints[i-1]
            x2, y2, _ = waypoints[i]
            distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            self.total_distance += distance
        
        # Estimate coverage (simplified)
        area_covered = self.area_width * self.area_height
        total_area = self.area_width * self.area_height
        self.coverage_percentage = (area_covered / total_area) * 100
        
        # Estimate time (assuming 0.5 m/s average speed)
        average_speed = 0.5  # m/s
        self.estimated_time = self.total_distance / average_speed / 60  # minutes
    
    def publish_coverage_path(self):
        """Publish current coverage path"""
        if self.current_path is None:
            return
            
        # Update timestamp
        self.current_path.header.stamp = self.get_clock().now().to_msg()
        
        # Publish path
        self.path_pub.publish(self.current_path)
        
        # Publish statistics
        stats_msg = String()
        stats_msg.data = (f"Pattern: {self.pattern_type}, "
                         f"Distance: {self.total_distance:.1f}m, "
                         f"Coverage: {self.coverage_percentage:.1f}%, "
                         f"Est. Time: {self.estimated_time:.1f}min, "
                         f"Waypoints: {len(self.current_path.poses)}")
        self.stats_pub.publish(stats_msg)
        
        # Publish visualizations
        self.publish_visualizations()
    
    def publish_visualizations(self):
        """Publish visualization markers"""
        markers = MarkerArray()
        
        # Area boundary marker
        boundary_marker = Marker()
        boundary_marker.header.frame_id = 'map'
        boundary_marker.header.stamp = self.get_clock().now().to_msg()
        boundary_marker.id = 0
        boundary_marker.type = Marker.LINE_STRIP
        boundary_marker.action = Marker.ADD
        
        # Area boundary points
        boundary_points = [
            Point(x=float(self.x_min), y=float(self.y_min), z=0.0),
            Point(x=float(self.x_max), y=float(self.y_min), z=0.0),
            Point(x=float(self.x_max), y=float(self.y_max), z=0.0),
            Point(x=float(self.x_min), y=float(self.y_max), z=0.0),
            Point(x=float(self.x_min), y=float(self.y_min), z=0.0)  # Close loop
        ]
        boundary_marker.points = boundary_points
        
        boundary_marker.scale.x = 0.1  # Line width
        boundary_marker.color.r = 1.0
        boundary_marker.color.g = 1.0
        boundary_marker.color.b = 0.0  # Yellow
        boundary_marker.color.a = 1.0
        
        markers.markers.append(boundary_marker)
        
        # Path waypoint markers
        if self.current_path and len(self.current_path.poses) > 0:
            for i, pose in enumerate(self.current_path.poses):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i + 10  # Offset to avoid collision with boundary
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                marker.pose = pose.pose
                marker.pose.position.z = 0.1  # Slightly above ground
                
                marker.scale.x = 0.8  # Length
                marker.scale.y = 0.2  # Width
                marker.scale.z = 0.2  # Height
                
                # Color gradient from start (green) to end (red)
                ratio = i / max(1, len(self.current_path.poses) - 1)
                marker.color.r = float(ratio)
                marker.color.g = float(1.0 - ratio)
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                markers.markers.append(marker)
        
        # Pattern info text marker
        info_marker = Marker()
        info_marker.header.frame_id = 'map'
        info_marker.header.stamp = self.get_clock().now().to_msg()
        info_marker.id = 1000
        info_marker.type = Marker.TEXT_VIEW_FACING
        info_marker.action = Marker.ADD
        
        info_marker.pose.position.x = float(self.area_center_x)
        info_marker.pose.position.y = float(self.y_max + 2.0)
        info_marker.pose.position.z = 2.0
        info_marker.pose.orientation.w = 1.0
        
        info_marker.scale.z = 1.0
        info_marker.color.r = 1.0
        info_marker.color.g = 1.0
        info_marker.color.b = 1.0
        info_marker.color.a = 1.0
        
        info_marker.text = (f"{self.pattern_type.upper()} PATTERN\n"
                           f"{len(self.current_path.poses) if self.current_path else 0} waypoints\n"
                           f"{self.total_distance:.1f}m total")
        
        markers.markers.append(info_marker)
        
        # Publish all markers
        self.markers_pub.publish(markers)
        self.boundary_pub.publish(markers)
    
    def cycle_pattern(self):
        """Cycle through different patterns for demonstration"""
        current_index = self.available_patterns.index(self.pattern_type)
        next_index = (current_index + 1) % len(self.available_patterns)
        self.pattern_type = self.available_patterns[next_index]
        
        self.get_logger().info(f'Switching to {self.pattern_type} pattern')
        self.generate_coverage_path()
    
    def execute_coverage_path(self):
        """Execute coverage path using Nav2 (if enabled)"""
        if not self.auto_execute or not self.current_path:
            return
            
        self.get_logger().info('Nav2 execution disabled for testing - path published for visualization only')
    
    # def nav_goal_response_callback(self, future):
    #     """Handle Nav2 goal response"""
    #     goal_handle = future.result()
    #     
    #     if not goal_handle.accepted:
    #         self.get_logger().error('Coverage path goal rejected')
    #         return
    #     
    #     self.get_logger().info('Coverage path goal accepted, executing...')
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.nav_result_callback)
    # 
    # def nav_result_callback(self, future):
    #     """Handle Nav2 execution result"""
    #     result = future.result().result
    #     self.get_logger().info(f'Coverage path completed: {result}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = SimpleCoveragePlanner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in simple coverage planner: {e}')
    finally:
        if 'planner' in locals():
            planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()