#!/usr/bin/env python3
"""
Simplified Obstacle Detection Node - Using Standard Messages  
Real-time obstacle detection using depth data from D455
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

# Standard ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int32, Header
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

class SimpleObstacleDetector(Node):
    def __init__(self):
        super().__init__('simple_obstacle_detector')
        
        self.bridge = CvBridge()
        
        # QoS profile
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Declare and get detection parameters
        self.declare_parameter('simple_obstacle_detector.min_obstacle_height', 0.10)
        self.declare_parameter('simple_obstacle_detector.max_detection_distance', 10.0)
        self.declare_parameter('simple_obstacle_detector.min_detection_distance', 0.3)
        self.declare_parameter('simple_obstacle_detector.ground_plane_threshold', 0.05)
        self.declare_parameter('simple_obstacle_detector.min_cluster_size', 50)
        self.declare_parameter('simple_obstacle_detector.detection_rate', 15.0)
        self.declare_parameter('simple_obstacle_detector.ground_region_ratio', 0.6)
        self.declare_parameter('simple_obstacle_detector.close_obstacle_threshold', 2.0)
        self.declare_parameter('simple_obstacle_detector.medium_distance_threshold', 5.0)
        self.declare_parameter('simple_obstacle_detector.marker_lifetime', 1.0)
        self.declare_parameter('simple_obstacle_detector.marker_alpha', 0.7)
        self.declare_parameter('simple_obstacle_detector.min_marker_size', 0.1)
        self.declare_parameter('simple_obstacle_detector.fixed_marker_height', 0.2)
        
        # Get parameter values
        self.min_obstacle_height = self.get_parameter('simple_obstacle_detector.min_obstacle_height').get_parameter_value().double_value
        self.max_detection_distance = self.get_parameter('simple_obstacle_detector.max_detection_distance').get_parameter_value().double_value
        self.min_detection_distance = self.get_parameter('simple_obstacle_detector.min_detection_distance').get_parameter_value().double_value
        self.ground_plane_threshold = self.get_parameter('simple_obstacle_detector.ground_plane_threshold').get_parameter_value().double_value
        self.min_cluster_size = self.get_parameter('simple_obstacle_detector.min_cluster_size').get_parameter_value().integer_value
        self.detection_rate = self.get_parameter('simple_obstacle_detector.detection_rate').get_parameter_value().double_value
        self.ground_region_ratio = self.get_parameter('simple_obstacle_detector.ground_region_ratio').get_parameter_value().double_value
        self.close_obstacle_threshold = self.get_parameter('simple_obstacle_detector.close_obstacle_threshold').get_parameter_value().double_value
        self.medium_distance_threshold = self.get_parameter('simple_obstacle_detector.medium_distance_threshold').get_parameter_value().double_value
        self.marker_lifetime = self.get_parameter('simple_obstacle_detector.marker_lifetime').get_parameter_value().double_value
        self.marker_alpha = self.get_parameter('simple_obstacle_detector.marker_alpha').get_parameter_value().double_value
        self.min_marker_size = self.get_parameter('simple_obstacle_detector.min_marker_size').get_parameter_value().double_value
        self.fixed_marker_height = self.get_parameter('simple_obstacle_detector.fixed_marker_height').get_parameter_value().double_value
        
        # Initialize variables
        self.latest_depth_image = None
        self.latest_color_image = None
        self.camera_info = None
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            self.image_qos
        )
        
        self.color_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/color/image_raw',
            self.color_callback,
            self.image_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/d455/d455_camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers using standard messages
        self.obstacle_count_pub = self.create_publisher(
            Int32,
            '~/obstacle_count',
            10
        )
        
        self.obstacle_status_pub = self.create_publisher(
            String,
            '~/obstacle_status',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '~/debug_image',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '~/obstacle_markers',
            10
        )
        
        # Detection timer with configurable rate
        timer_period = 1.0 / self.detection_rate
        self.detection_timer = self.create_timer(timer_period, self.detection_callback)
        
        self.get_logger().info('Simple Obstacle Detector Node initialized')
        self.get_logger().info(f'Detection range: {self.min_detection_distance:.1f}m - {self.max_detection_distance:.1f}m')
    
    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def color_callback(self, msg):
        """Callback for color image"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
    
    def camera_info_callback(self, msg):
        """Callback for camera info"""
        self.camera_info = msg
    
    def detect_obstacles(self, depth_image):
        """Detect obstacles in depth image"""
        if depth_image is None or self.camera_info is None:
            return []
        
        try:
            # Convert depth to meters
            depth_m = depth_image.astype(np.float32) / 1000.0
            
            # Create valid depth mask
            valid_mask = (depth_m > self.min_detection_distance) & (depth_m < self.max_detection_distance)
            
            # Estimate ground plane with configurable region ratio
            h, w = depth_m.shape
            ground_start_row = int(self.ground_region_ratio * h)
            ground_region = depth_m[ground_start_row:h, :]
            ground_valid = valid_mask[ground_start_row:h, :]
            
            ground_depths = ground_region[ground_valid]
            if len(ground_depths) < 100:
                return []
            
            ground_level = np.median(ground_depths)
            
            # Find points above ground
            height_above_ground = ground_level - depth_m
            obstacle_mask = (height_above_ground > self.min_obstacle_height) & valid_mask
            
            # Find connected components (obstacles)
            num_labels, labels = cv2.connectedComponents(obstacle_mask.astype(np.uint8))
            
            obstacles = []
            
            for label in range(1, num_labels):  # Skip background (0)
                cluster_mask = (labels == label)
                cluster_points = np.sum(cluster_mask)
                
                if cluster_points < self.min_cluster_size:
                    continue
                
                # Get obstacle properties
                y_coords, x_coords = np.where(cluster_mask)
                if len(y_coords) == 0:
                    continue
                
                # Calculate obstacle bounding box
                x_min, x_max = np.min(x_coords), np.max(x_coords)
                y_min, y_max = np.min(y_coords), np.max(y_coords)
                
                # Estimate 3D position (simplified)
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                obstacle_depth = depth_m[center_y, center_x]
                
                if obstacle_depth <= 0:
                    continue
                
                # Calculate obstacle size
                width_px = x_max - x_min
                height_px = y_max - y_min
                
                # Convert pixel size to real-world size (rough approximation)
                pixel_size_at_depth = obstacle_depth / self.camera_info.k[0]  # Approximate
                width_m = width_px * pixel_size_at_depth
                height_m = height_px * pixel_size_at_depth
                
                obstacles.append({
                    'position': [center_x, center_y, obstacle_depth],
                    'size': [width_m, height_m],
                    'pixel_coords': (x_min, y_min, x_max, y_max),
                    'depth': obstacle_depth,
                    'point_count': cluster_points
                })
            
            return obstacles
            
        except Exception as e:
            self.get_logger().debug(f'Error in obstacle detection: {e}')
            return []
    
    def create_debug_image(self, color_image, obstacles):
        """Create debug visualization"""
        if color_image is None:
            return None
        
        debug_image = color_image.copy()
        
        for i, obstacle in enumerate(obstacles):
            x_min, y_min, x_max, y_max = obstacle['pixel_coords']
            
            # Draw bounding box
            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
            
            # Add obstacle info
            depth = obstacle['depth']
            size = obstacle['size']
            label = f"Obs {i+1}: {depth:.1f}m"
            cv2.putText(debug_image, label, (x_min, y_min - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Add summary info
        cv2.putText(debug_image, f"Obstacles: {len(obstacles)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_image
    
    def create_markers(self, obstacles):
        """Create RViz markers for obstacles"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "d455_color_optical_frame"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position (convert from image coordinates to 3D)
            x_px, y_px, depth = obstacle['position']
            
            # Convert pixel coordinates to 3D (simplified)
            if self.camera_info:
                x_3d = (x_px - self.camera_info.k[2]) * depth / self.camera_info.k[0]
                y_3d = (y_px - self.camera_info.k[5]) * depth / self.camera_info.k[4]
            else:
                x_3d, y_3d = 0, 0
            
            marker.pose.position.x = float(x_3d)
            marker.pose.position.y = float(y_3d)  
            marker.pose.position.z = float(depth)
            marker.pose.orientation.w = 1.0
            
            # Size with configurable minimums
            width, height = obstacle['size']
            marker.scale.x = max(width, self.min_marker_size)
            marker.scale.y = max(height, self.min_marker_size)
            marker.scale.z = self.fixed_marker_height
            
            # Color based on configurable distance thresholds
            if depth < self.close_obstacle_threshold:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # Red - close
            elif depth < self.medium_distance_threshold:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # Yellow - medium
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # Green - far
            
            marker.color.a = self.marker_alpha
            marker.lifetime.sec = int(self.marker_lifetime)
            
            marker_array.markers.append(marker)
        
        return marker_array
    
    def detection_callback(self):
        """Main detection callback"""
        if self.latest_depth_image is None:
            return
        
        try:
            # Detect obstacles
            obstacles = self.detect_obstacles(self.latest_depth_image)
            
            # Publish obstacle count
            count_msg = Int32()
            count_msg.data = len(obstacles)
            self.obstacle_count_pub.publish(count_msg)
            
            # Publish status
            status_msg = String()
            if len(obstacles) == 0:
                status_msg.data = "No obstacles detected"
            else:
                close_obstacles = sum(1 for obs in obstacles if obs['depth'] < self.close_obstacle_threshold)
                status_msg.data = f"{len(obstacles)} obstacles detected, {close_obstacles} close"
            self.obstacle_status_pub.publish(status_msg)
            
            # Publish debug image
            if self.latest_color_image is not None:
                debug_image = self.create_debug_image(self.latest_color_image, obstacles)
                if debug_image is not None:
                    try:
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                        debug_msg.header.stamp = self.get_clock().now().to_msg()
                        debug_msg.header.frame_id = "d455_color_optical_frame"
                        self.debug_image_pub.publish(debug_msg)
                    except Exception as e:
                        self.get_logger().debug(f'Failed to publish debug image: {e}')
            
            # Publish RViz markers
            markers = self.create_markers(obstacles)
            self.marker_pub.publish(markers)
            
            # Log detection results with configurable threshold
            if len(obstacles) > 0:
                close_count = sum(1 for obs in obstacles if obs['depth'] < self.close_obstacle_threshold)
                self.get_logger().info(f'Detected {len(obstacles)} obstacles, {close_count} within {self.close_obstacle_threshold}m')
            
        except Exception as e:
            self.get_logger().error(f'Error in obstacle detection callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        obstacle_detector = SimpleObstacleDetector()
        rclpy.spin(obstacle_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in obstacle detector: {e}')
    finally:
        if 'obstacle_detector' in locals():
            obstacle_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()