#!/usr/bin/env python3
"""
Coverage Path Planner for D455 Lawn Mowing
Generates systematic mowing patterns based on grass coverage costmaps
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np
import cv2
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN
import math

# ROS 2 message types
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateThroughPoses
# Manual quaternion conversion functions

class CoveragePathPlanner(Node):
    def __init__(self):
        super().__init__('coverage_path_planner')
        
        # Planning parameters
        self.declare_parameter('mowing_width', 0.6)        # Mower cutting width (meters)
        self.declare_parameter('overlap_ratio', 0.2)       # Overlap between passes (0-1)
        self.declare_parameter('min_grass_coverage', 0.1)  # Minimum grass % to mow
        self.declare_parameter('pattern_type', 'boustrophedon')  # Pattern type
        self.declare_parameter('planning_frequency', 0.5)  # How often to replan (Hz)
        
        # Get parameters
        self.mowing_width = self.get_parameter('mowing_width').value
        self.overlap_ratio = self.get_parameter('overlap_ratio').value  
        self.min_grass_coverage = self.get_parameter('min_grass_coverage').value
        self.pattern_type = self.get_parameter('pattern_type').value
        self.planning_frequency = self.get_parameter('planning_frequency').value
        
        # Calculate effective strip width with overlap
        self.strip_width = self.mowing_width * (1.0 - self.overlap_ratio)
        
        # Current costmap data
        self.current_costmap = None
        self.costmap_metadata = None
        
        # Current robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Coverage tracking
        self.covered_areas = None
        self.priority_areas = None
        
        # Setup subscribers and publishers
        self.setup_communication()
        
        # Planning timer
        self.planning_timer = self.create_timer(1.0/self.planning_frequency, self.planning_callback)
        
        # Action client for navigation
        self.nav_action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        self.get_logger().info('Coverage Path Planner initialized')
        self.get_logger().info(f'Mowing width: {self.mowing_width}m, Strip width: {self.strip_width}m')
        self.get_logger().info(f'Pattern: {self.pattern_type}, Min grass: {self.min_grass_coverage}%')
    
    def setup_communication(self):
        """Setup ROS communication"""
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lawn_care/combined_costmap',
            self.costmap_callback,
            10
        )
        
        self.grass_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/lawn_care/grass_coverage_costmap',
            self.grass_costmap_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/lawn_care/coverage_path',
            10
        )
        
        self.waypoints_pub = self.create_publisher(
            MarkerArray,
            '/lawn_care/coverage_waypoints',
            10
        )
        
        self.coverage_markers_pub = self.create_publisher(
            MarkerArray,
            '/lawn_care/coverage_visualization',
            10
        )
        
        # TF buffer for robot pose
        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    def costmap_callback(self, msg):
        """Update current costmap"""
        self.current_costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.costmap_metadata = msg.info
    
    def grass_costmap_callback(self, msg):
        """Update grass coverage data"""
        grass_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # Initialize coverage tracking if needed
        if self.covered_areas is None:
            self.covered_areas = np.zeros_like(grass_map, dtype=bool)
            
        # Update priority areas (high grass coverage areas)
        self.priority_areas = (grass_map / 100.0) > self.min_grass_coverage
    
    def get_robot_pose(self):
        """Get current robot pose"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            q = transform.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            return True
            
        except Exception as e:
            self.get_logger().debug(f'Could not get robot pose: {e}')
            return False
    
    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map coordinates"""
        if self.costmap_metadata is None:
            return None, None
            
        map_x = int((world_x - self.costmap_metadata.origin.position.x) / 
                   self.costmap_metadata.resolution)
        map_y = int((world_y - self.costmap_metadata.origin.position.y) / 
                   self.costmap_metadata.resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Convert map coordinates to world coordinates"""
        if self.costmap_metadata is None:
            return None, None
            
        world_x = map_x * self.costmap_metadata.resolution + \
                  self.costmap_metadata.origin.position.x
        world_y = map_y * self.costmap_metadata.resolution + \
                  self.costmap_metadata.origin.position.y
        return world_x, world_y
    
    def find_mowable_regions(self):
        """Identify regions that need mowing"""
        if self.current_costmap is None or self.priority_areas is None:
            return []
        
        # Find areas with grass coverage that haven't been mowed recently
        mowable_mask = self.priority_areas & (~self.covered_areas)
        
        # Remove obstacles and high-cost areas
        safe_mask = self.current_costmap < 50  # Below neutral cost
        mowable_mask = mowable_mask & safe_mask
        
        # Find connected regions
        num_labels, labels = cv2.connectedComponents(mowable_mask.astype(np.uint8))
        
        regions = []
        for label in range(1, num_labels):
            region_mask = (labels == label)
            region_size = np.sum(region_mask)
            
            # Filter out small regions
            min_region_size = int((1.0 / self.costmap_metadata.resolution) ** 2)  # 1m² minimum
            if region_size < min_region_size:
                continue
            
            # Get region bounding box and centroid
            coords = np.where(region_mask)
            y_coords, x_coords = coords
            
            bbox = {
                'x_min': np.min(x_coords),
                'x_max': np.max(x_coords),
                'y_min': np.min(y_coords),
                'y_max': np.max(y_coords)
            }
            
            centroid = (np.mean(x_coords), np.mean(y_coords))
            
            regions.append({
                'mask': region_mask,
                'bbox': bbox,
                'centroid': centroid,
                'size': region_size,
                'label': label
            })
        
        return regions
    
    def generate_boustrophedon_path(self, region):
        """Generate boustrophedon (back-and-forth) mowing pattern"""
        if self.costmap_metadata is None:
            return []
        
        bbox = region['bbox']
        
        # Calculate number of strips needed
        region_width = (bbox['x_max'] - bbox['x_min']) * self.costmap_metadata.resolution
        num_strips = max(1, int(region_width / self.strip_width) + 1)
        
        waypoints = []
        
        # Generate parallel strips
        for strip_idx in range(num_strips):
            # Calculate strip position
            strip_fraction = strip_idx / max(1, num_strips - 1) if num_strips > 1 else 0.5
            strip_x = bbox['x_min'] + strip_fraction * (bbox['x_max'] - bbox['x_min'])
            
            # Find intersection points with region boundary
            intersections = []
            
            # Check intersection with region mask along this strip
            for map_y in range(bbox['y_min'], bbox['y_max'] + 1):
                if (strip_x < region['mask'].shape[1] and map_y < region['mask'].shape[0] and
                    region['mask'][map_y, int(strip_x)]):
                    world_x, world_y = self.map_to_world(strip_x, map_y)
                    if world_x is not None and world_y is not None:
                        intersections.append((world_x, world_y, map_y))
            
            if len(intersections) < 2:
                continue
                
            # Sort intersections by y-coordinate
            intersections.sort(key=lambda p: p[2])
            
            # Create strip path (alternate direction for boustrophedon)
            if strip_idx % 2 == 0:
                # Forward direction
                start_point = intersections[0]
                end_point = intersections[-1]
            else:
                # Reverse direction
                start_point = intersections[-1]
                end_point = intersections[0]
            
            waypoints.append((start_point[0], start_point[1]))
            waypoints.append((end_point[0], end_point[1]))
        
        return waypoints
    
    def generate_spiral_path(self, region):
        """Generate spiral mowing pattern (alternative to boustrophedon)"""
        # Simplified spiral pattern - start from outside and work inward
        bbox = region['bbox']
        
        # Convert to world coordinates
        x_min, y_min = self.map_to_world(bbox['x_min'], bbox['y_min'])
        x_max, y_max = self.map_to_world(bbox['x_max'], bbox['y_max'])
        
        if None in [x_min, y_min, x_max, y_max]:
            return []
        
        waypoints = []
        margin = self.strip_width
        
        current_x_min = x_min + margin
        current_x_max = x_max - margin
        current_y_min = y_min + margin
        current_y_max = y_max - margin
        
        while (current_x_max - current_x_min > self.strip_width and 
               current_y_max - current_y_min > self.strip_width):
            
            # Bottom edge (left to right)
            waypoints.append((current_x_min, current_y_min))
            waypoints.append((current_x_max, current_y_min))
            
            # Right edge (bottom to top)
            waypoints.append((current_x_max, current_y_max))
            
            # Top edge (right to left)
            waypoints.append((current_x_min, current_y_max))
            
            # Left edge (top to bottom)
            waypoints.append((current_x_min, current_y_min + margin))
            
            # Shrink the boundary for next spiral
            current_x_min += margin
            current_x_max -= margin
            current_y_min += margin  
            current_y_max -= margin
        
        return waypoints
    
    def optimize_path_order(self, regions_with_paths):
        """Optimize the order of regions to minimize travel distance"""
        if not regions_with_paths:
            return []
        
        # Get robot position
        if not self.get_robot_pose():
            return regions_with_paths  # Return unoptimized if no pose
        
        current_pos = np.array([self.robot_x, self.robot_y])
        
        # Calculate distances to each region start
        region_starts = []
        for region_data in regions_with_paths:
            path = region_data['path']
            if path:
                start_pos = np.array(path[0])
                distance = np.linalg.norm(current_pos - start_pos)
                region_starts.append((distance, region_data))
        
        # Sort by distance (simple nearest-neighbor)
        region_starts.sort(key=lambda x: x[0])
        
        return [region_data for _, region_data in region_starts]
    
    def create_nav2_path(self, waypoints):
        """Convert waypoints to Nav2 Path message"""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Calculate orientation based on direction to next waypoint
            if i < len(waypoints) - 1:
                next_x, next_y = waypoints[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = 0.0  # Default orientation for last waypoint
            
            # Manual quaternion from yaw
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            q = [0, 0, sy, cy]  # [x, y, z, w]
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path.poses.append(pose)
        
        return path
    
    def planning_callback(self):
        """Main planning callback"""
        
        # Check if we have necessary data
        if self.current_costmap is None or self.priority_areas is None:
            return
        
        # Find mowable regions
        regions = self.find_mowable_regions()
        
        if not regions:
            self.get_logger().debug('No mowable regions found')
            return
        
        # Generate paths for each region
        regions_with_paths = []
        
        for region in regions:
            if self.pattern_type == 'boustrophedon':
                waypoints = self.generate_boustrophedon_path(region)
            elif self.pattern_type == 'spiral':
                waypoints = self.generate_spiral_path(region)
            else:
                self.get_logger().warn(f'Unknown pattern type: {self.pattern_type}')
                continue
            
            if waypoints:
                regions_with_paths.append({
                    'region': region,
                    'path': waypoints,
                    'priority': region['size'] * np.sum(self.priority_areas & region['mask'])
                })
        
        if not regions_with_paths:
            return
        
        # Sort regions by priority (largest high-grass areas first)
        regions_with_paths.sort(key=lambda x: x['priority'], reverse=True)
        
        # Optimize path order
        optimized_regions = self.optimize_path_order(regions_with_paths[:3])  # Top 3 regions
        
        # Combine all waypoints
        all_waypoints = []
        for region_data in optimized_regions:
            all_waypoints.extend(region_data['path'])
        
        if all_waypoints:
            # Create and publish path
            path_msg = self.create_nav2_path(all_waypoints)
            self.path_pub.publish(path_msg)
            
            # Publish visualization
            self.publish_visualization(regions_with_paths, all_waypoints)
            
            self.get_logger().info(f'Generated coverage path with {len(all_waypoints)} waypoints across {len(optimized_regions)} regions')
    
    def publish_visualization(self, regions_with_paths, waypoints):
        """Publish visualization markers"""
        
        marker_array = MarkerArray()
        
        # Region markers
        for i, region_data in enumerate(regions_with_paths):
            region = region_data['region']
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position at region centroid
            world_x, world_y = self.map_to_world(region['centroid'][0], region['centroid'][1])
            if world_x is not None:
                marker.pose.position.x = world_x
                marker.pose.position.y = world_y
                marker.pose.position.z = 0.5
                marker.pose.orientation.w = 1.0
                
                marker.scale.x = 2.0
                marker.scale.y = 2.0
                marker.scale.z = 1.0
                
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7
                
                marker_array.markers.append(marker)
        
        # Waypoint markers
        for i, (x, y) in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = len(regions_with_paths) + i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color gradient from start (blue) to end (red)
            ratio = i / max(1, len(waypoints) - 1)
            marker.color.r = ratio
            marker.color.g = 0.0
            marker.color.b = 1.0 - ratio
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.waypoints_pub.publish(marker_array)
    
    def execute_coverage_path(self, path):
        """Execute coverage path using Nav2"""
        
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = path.poses
        
        self.get_logger().info(f'Executing coverage path with {len(path.poses)} poses')
        
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Coverage path goal rejected')
            return
        
        self.get_logger().info('Coverage path goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle Nav2 execution result"""
        result = future.result().result
        self.get_logger().info(f'Coverage path completed with result: {result}')
        
        # Mark areas as covered (simplified - could use actual robot path)
        # In a real implementation, you'd track the actual mowed areas


def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = CoveragePathPlanner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in coverage path planner: {e}')
    finally:
        if 'planner' in locals():
            planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()