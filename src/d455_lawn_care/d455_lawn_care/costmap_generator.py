#!/usr/bin/env python3
"""
D455 Lawn Care Costmap Generator for Nav2
Converts grass detection and obstacle data into Nav2-compatible costmaps
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs

# ROS 2 message types
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32, Header
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker, MarkerArray

class LawnCareCostmapGenerator(Node):
    def __init__(self):
        super().__init__('lawn_care_costmap_generator')
        
        self.bridge = CvBridge()
        
        # Costmap parameters
        self.declare_parameter('map_width', 50.0)          # meters
        self.declare_parameter('map_height', 50.0)         # meters  
        self.declare_parameter('map_resolution', 0.1)      # meters per pixel
        self.declare_parameter('map_origin_x', -25.0)      # meters
        self.declare_parameter('map_origin_y', -25.0)      # meters
        self.declare_parameter('update_rate', 2.0)         # Hz
        self.declare_parameter('grass_decay_rate', 0.95)   # How fast grass data decays
        self.declare_parameter('obstacle_decay_rate', 0.90) # How fast obstacle data decays
        
        # Get parameters
        self.map_width = float(self.get_parameter('map_width').value)
        self.map_height = float(self.get_parameter('map_height').value)
        self.map_resolution = float(self.get_parameter('map_resolution').value)
        self.map_origin_x = float(self.get_parameter('map_origin_x').value)
        self.map_origin_y = float(self.get_parameter('map_origin_y').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.grass_decay_rate = float(self.get_parameter('grass_decay_rate').value)
        self.obstacle_decay_rate = float(self.get_parameter('obstacle_decay_rate').value)
        
        # Calculate map dimensions in pixels
        self.map_width_px = int(self.map_width / self.map_resolution)
        self.map_height_px = int(self.map_height / self.map_resolution)
        
        # Initialize costmaps
        self.grass_coverage_map = np.zeros((self.map_height_px, self.map_width_px), dtype=np.float32)
        self.obstacle_map = np.zeros((self.map_height_px, self.map_width_px), dtype=np.float32)
        self.combined_costmap = np.zeros((self.map_height_px, self.map_width_px), dtype=np.int8)
        
        # Robot state (ensure float type)
        self.robot_x = float(0.0)
        self.robot_y = float(0.0)
        self.robot_yaw = float(0.0)
        
        # Latest detection data
        self.latest_grass_coverage = 0.0
        self.latest_obstacle_count = 0
        self.camera_fov_angle = 60.0  # degrees
        self.camera_range = 5.0       # meters
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.setup_subscribers()
        self.setup_publishers()
        
        # Update timer
        self.update_timer = self.create_timer(1.0/self.update_rate, self.update_costmaps)
        
        self.get_logger().info('Lawn Care Costmap Generator initialized')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height}m at {self.map_resolution}m/px')
        self.get_logger().info(f'Map dimensions: {self.map_width_px}x{self.map_height_px}px')
    
    def setup_subscribers(self):
        """Setup subscribers for detection data"""
        
        # Grass detection data
        self.grass_coverage_sub = self.create_subscription(
            Float32,
            '/simple_grass_detector/grass_coverage_percentage',
            self.grass_coverage_callback,
            10
        )
        
        # Obstacle detection data
        self.obstacle_count_sub = self.create_subscription(
            Int32,
            '/simple_obstacle_detector/obstacle_count',
            self.obstacle_count_callback,
            10
        )
        
        # Camera image for spatial awareness (optional)
        self.depth_image_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/aligned_depth_to_color/image_raw',
            self.depth_image_callback,
            self.sensor_qos
        )
    
    def setup_publishers(self):
        """Setup publishers for costmap data"""
        
        # Grass coverage costmap
        self.grass_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/lawn_care/grass_coverage_costmap',
            10
        )
        
        # Obstacle costmap
        self.obstacle_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/lawn_care/obstacle_costmap',
            10
        )
        
        # Combined costmap for Nav2
        self.combined_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/lawn_care/combined_costmap',
            10
        )
        
        # Visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/lawn_care/costmap_markers',
            10
        )
    
    def grass_coverage_callback(self, msg):
        """Update grass coverage data"""
        self.latest_grass_coverage = msg.data
    
    def obstacle_count_callback(self, msg):
        """Update obstacle detection data"""
        self.latest_obstacle_count = msg.data
    
    def depth_image_callback(self, msg):
        """Process depth image for spatial mapping (optional enhancement)"""
        try:
            # Could use depth data for more precise spatial mapping
            # For now, we'll use the robot's position and detection data
            pass
        except Exception as e:
            self.get_logger().debug(f'Error processing depth image: {e}')
    
    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            
            # Convert quaternion to yaw
            q = transform.transform.rotation
            import math
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            return True
            
        except Exception as e:
            self.get_logger().debug(f'Could not get robot pose: {e}')
            return False
    
    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map pixel coordinates"""
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Convert map pixel coordinates to world coordinates"""
        world_x = map_x * self.map_resolution + self.map_origin_x
        world_y = map_y * self.map_resolution + self.map_origin_y
        return world_x, world_y
    
    def is_valid_map_coord(self, map_x, map_y):
        """Check if map coordinates are within bounds"""
        return 0 <= map_x < self.map_width_px and 0 <= map_y < self.map_height_px
    
    def update_grass_coverage_map(self):
        """Update grass coverage costmap based on current robot position and detection"""
        if not self.get_robot_pose():
            return
        
        # Get robot position in map coordinates
        robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
        
        if not self.is_valid_map_coord(robot_map_x, robot_map_y):
            return
        
        # Update grass coverage in camera field of view
        # Create a sector in front of the robot representing camera FOV
        import math
        
        # Camera FOV parameters
        fov_radius_px = int(self.camera_range / self.map_resolution)
        fov_half_angle = math.radians(self.camera_fov_angle / 2.0)
        
        # Create circular sector for camera FOV
        for dy in range(-fov_radius_px, fov_radius_px + 1):
            for dx in range(-fov_radius_px, fov_radius_px + 1):
                # Check if point is within camera range
                distance = math.sqrt(dx*dx + dy*dy)
                if distance > fov_radius_px:
                    continue
                
                # Check if point is within camera FOV angle
                if distance > 0:
                    angle_to_point = math.atan2(dy, dx)
                    angle_diff = abs(angle_to_point - self.robot_yaw)
                    # Normalize angle difference
                    while angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    angle_diff = abs(angle_diff)
                    
                    if angle_diff > fov_half_angle:
                        continue
                
                # Update map coordinates
                map_x = robot_map_x + dx
                map_y = robot_map_y + dy
                
                if self.is_valid_map_coord(map_x, map_y):
                    # Update grass coverage based on detection
                    # Higher grass coverage = lower cost (more attractive for mowing)
                    grass_intensity = self.latest_grass_coverage / 100.0
                    
                    # Use exponential decay for distance weighting
                    distance_weight = math.exp(-distance * 0.1)
                    
                    # Update grass coverage with weighted average
                    current_value = self.grass_coverage_map[map_y, map_x]
                    new_value = grass_intensity * distance_weight
                    
                    # Blend with existing data
                    alpha = 0.3  # Learning rate
                    self.grass_coverage_map[map_y, map_x] = (
                        alpha * new_value + (1 - alpha) * current_value
                    )
    
    def update_obstacle_map(self):
        """Update obstacle costmap based on detection data"""
        if not self.get_robot_pose():
            return
        
        # Get robot position in map coordinates
        robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)
        
        if not self.is_valid_map_coord(robot_map_x, robot_map_y):
            return
        
        # If obstacles detected, mark area in front of robot as having obstacles
        if self.latest_obstacle_count > 0:
            import math
            
            # Mark obstacle region (smaller, more precise than grass FOV)
            obstacle_radius_px = int(3.0 / self.map_resolution)  # 3 meter obstacle radius
            fov_half_angle = math.radians(30.0)  # Narrower FOV for obstacles
            
            for dy in range(-obstacle_radius_px, obstacle_radius_px + 1):
                for dx in range(-obstacle_radius_px, obstacle_radius_px + 1):
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance > obstacle_radius_px or distance < 0.5:
                        continue
                    
                    # Check angle
                    if distance > 0:
                        angle_to_point = math.atan2(dy, dx)
                        angle_diff = abs(angle_to_point - self.robot_yaw)
                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        angle_diff = abs(angle_diff)
                        
                        if angle_diff > fov_half_angle:
                            continue
                    
                    map_x = robot_map_x + dx
                    map_y = robot_map_y + dy
                    
                    if self.is_valid_map_coord(map_x, map_y):
                        # Higher obstacle count = higher cost (avoid area)
                        obstacle_intensity = min(1.0, self.latest_obstacle_count / 5.0)
                        distance_weight = math.exp(-distance * 0.2)
                        
                        current_value = self.obstacle_map[map_y, map_x]
                        new_value = obstacle_intensity * distance_weight
                        
                        # More aggressive learning for obstacles (safety)
                        alpha = 0.7
                        self.obstacle_map[map_y, map_x] = max(
                            current_value,
                            alpha * new_value + (1 - alpha) * current_value
                        )
    
    def update_costmaps(self):
        """Main costmap update loop"""
        
        # Apply decay to existing maps
        self.grass_coverage_map *= self.grass_decay_rate
        self.obstacle_map *= self.obstacle_decay_rate
        
        # Update maps based on current detections
        self.update_grass_coverage_map()
        self.update_obstacle_map()
        
        # Generate combined costmap
        self.generate_combined_costmap()
        
        # Publish costmaps
        self.publish_costmaps()
    
    def generate_combined_costmap(self):
        """Generate combined costmap for Nav2 navigation"""
        
        # Start with neutral cost
        combined = np.full_like(self.combined_costmap, 50, dtype=np.int8)  # Neutral cost
        
        # Process grass coverage (inverse cost - more grass = lower cost)
        grass_normalized = self.grass_coverage_map * 255
        grass_cost = np.clip(100 - grass_normalized * 0.5, 0, 100).astype(np.int8)
        
        # Process obstacles (high cost areas)
        obstacle_normalized = self.obstacle_map * 255
        obstacle_cost = np.clip(obstacle_normalized * 2, 0, 254).astype(np.int8)
        
        # Combine costs
        # Obstacles take priority (max operation)
        # Then adjust by grass coverage
        self.combined_costmap = np.maximum(grass_cost, obstacle_cost)
        
        # Mark lethal obstacles (if any)
        lethal_mask = obstacle_normalized > 200
        self.combined_costmap[lethal_mask] = 254  # Lethal cost
        
        # Unknown areas remain at neutral cost (50)
        unknown_mask = (self.grass_coverage_map == 0) & (self.obstacle_map == 0)
        self.combined_costmap[unknown_mask] = -1  # Unknown space
    
    def create_occupancy_grid_msg(self, costmap, frame_id, topic_name):
        """Create OccupancyGrid message from costmap array"""
        
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Map metadata
        msg.info.width = self.map_width_px
        msg.info.height = self.map_height_px
        msg.info.resolution = self.map_resolution
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Convert costmap to OccupancyGrid format
        if costmap.dtype == np.float32:
            # Convert float32 to int8 for OccupancyGrid
            data = np.clip(costmap * 100, 0, 100).astype(np.int8)
        else:
            data = costmap.astype(np.int8)
        
        # Flatten and convert to list (OccupancyGrid expects row-major order)
        msg.data = data.flatten().tolist()
        
        return msg
    
    def publish_costmaps(self):
        """Publish all costmap data"""
        
        timestamp = self.get_clock().now()
        
        # Grass coverage costmap
        grass_msg = self.create_occupancy_grid_msg(
            self.grass_coverage_map, 'map', 'grass_coverage'
        )
        self.grass_costmap_pub.publish(grass_msg)
        
        # Obstacle costmap
        obstacle_msg = self.create_occupancy_grid_msg(
            self.obstacle_map, 'map', 'obstacles'
        )
        self.obstacle_costmap_pub.publish(obstacle_msg)
        
        # Combined costmap for Nav2
        combined_msg = self.create_occupancy_grid_msg(
            self.combined_costmap, 'map', 'combined'
        )
        self.combined_costmap_pub.publish(combined_msg)
        
        # Publish visualization markers
        self.publish_markers()
    
    def publish_markers(self):
        """Publish visualization markers for RViz"""
        
        marker_array = MarkerArray()
        
        # Robot position marker
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.id = 0
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        
        robot_marker.pose.position.x = float(self.robot_x)
        robot_marker.pose.position.y = float(self.robot_y)
        robot_marker.pose.position.z = float(0.1)
        
        # Set orientation from yaw (manual quaternion conversion)
        import math
        cy = math.cos(self.robot_yaw * 0.5)
        sy = math.sin(self.robot_yaw * 0.5)
        q = [0, 0, sy, cy]  # [x, y, z, w]
        robot_marker.pose.orientation.x = float(q[0])
        robot_marker.pose.orientation.y = float(q[1])
        robot_marker.pose.orientation.z = float(q[2])
        robot_marker.pose.orientation.w = float(q[3])
        
        robot_marker.scale.x = 1.0
        robot_marker.scale.y = 0.3
        robot_marker.scale.z = 0.3
        
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.8
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0
        
        marker_array.markers.append(robot_marker)
        
        # Detection info text marker
        info_marker = Marker()
        info_marker.header.frame_id = 'map'
        info_marker.header.stamp = self.get_clock().now().to_msg()
        info_marker.id = 1
        info_marker.type = Marker.TEXT_VIEW_FACING
        info_marker.action = Marker.ADD
        
        info_marker.pose.position.x = float(self.robot_x)
        info_marker.pose.position.y = float(self.robot_y + 1.0)
        info_marker.pose.position.z = float(1.0)
        info_marker.pose.orientation.w = 1.0
        
        info_marker.scale.z = 0.5
        info_marker.color.r = 1.0
        info_marker.color.g = 1.0
        info_marker.color.b = 1.0
        info_marker.color.a = 1.0
        
        info_marker.text = f"Grass: {self.latest_grass_coverage:.1f}%\nObstacles: {self.latest_obstacle_count}"
        
        marker_array.markers.append(info_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        costmap_generator = LawnCareCostmapGenerator()
        rclpy.spin(costmap_generator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in costmap generator: {e}')
    finally:
        if 'costmap_generator' in locals():
            costmap_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()