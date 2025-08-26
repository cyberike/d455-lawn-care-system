#!/usr/bin/env python3
"""
Obstacle Detection Node for D455 Lawn Care
Real-time obstacle detection and classification using depth and RGB data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from collections import defaultdict

# ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, Vector3, Polygon, Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from d455_lawn_care.msg import Obstacle, ObstacleArray

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # QoS profiles
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Obstacle detection thresholds
                ('min_obstacle_height', 0.10),        # 10cm minimum height
                ('max_detection_distance', 10.0),     # 10m maximum detection distance
                ('min_detection_distance', 0.3),      # 30cm minimum detection distance
                ('ground_plane_threshold', 0.05),     # 5cm ground plane tolerance
                
                # Clustering parameters
                ('cluster_tolerance', 0.05),          # 5cm clustering tolerance
                ('min_cluster_size', 50),             # Minimum points per obstacle
                ('max_cluster_size', 5000),           # Maximum points per obstacle
                
                # Obstacle classification
                ('tree_height_threshold', 1.0),       # 1m height for tree classification
                ('rock_aspect_ratio_threshold', 0.8), # Width/height ratio for rocks
                ('furniture_size_threshold', 0.5),    # Minimum size for furniture
                
                # Tracking parameters
                ('obstacle_timeout', 5.0),            # Seconds before obstacle timeout
                ('position_tolerance', 0.2),          # 20cm position tolerance for tracking
                ('min_detections_for_confirmation', 3), # Minimum detections to confirm obstacle
                
                # Publishing settings
                ('publish_markers', True),             # Whether to publish visualization markers
                ('publish_debug_images', True),       # Whether to publish debug images
                ('detection_frequency', 15.0),        # Detection frequency in Hz
                ('frame_id', 'd455_color_optical_frame'), # Frame ID
                
                # Safety parameters
                ('safety_buffer_distance', 0.5),      # Safety buffer around obstacles
                ('danger_zone_distance', 1.0),        # Distance for high danger level
            ]
        )
        
        # Get parameters
        self.update_parameters()
        
        # Initialize variables
        self.latest_depth_image = None
        self.latest_color_image = None
        self.camera_info = None
        self.detection_active = True
        
        # Obstacle tracking
        self.tracked_obstacles = {}
        self.next_obstacle_id = 1
        
        # Create subscribers
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
        
        # Create publishers
        self.obstacle_array_pub = self.create_publisher(
            ObstacleArray,
            '~/obstacles',
            10
        )
        
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                '~/obstacle_markers',
                10
            )
        
        if self.publish_debug_images:
            self.debug_image_pub = self.create_publisher(
                Image,
                '~/debug_image',
                10
            )
        
        # Create detection timer
        self.detection_timer = self.create_timer(
            1.0 / self.detection_frequency,
            self.detection_callback
        )
        
        self.get_logger().info('Obstacle Detector Node initialized')
        self.get_logger().info(f'Detection range: {self.min_detection_distance:.1f}m - {self.max_detection_distance:.1f}m')
        self.get_logger().info(f'Minimum obstacle height: {self.min_obstacle_height:.2f}m')
    
    def update_parameters(self):
        """Update parameters from ROS parameter server"""
        # Detection thresholds
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').value
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        self.min_detection_distance = self.get_parameter('min_detection_distance').value
        self.ground_plane_threshold = self.get_parameter('ground_plane_threshold').value
        
        # Clustering
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        
        # Classification
        self.tree_height_threshold = self.get_parameter('tree_height_threshold').value
        self.rock_aspect_ratio_threshold = self.get_parameter('rock_aspect_ratio_threshold').value
        self.furniture_size_threshold = self.get_parameter('furniture_size_threshold').value
        
        # Tracking
        self.obstacle_timeout = self.get_parameter('obstacle_timeout').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.min_detections_for_confirmation = self.get_parameter('min_detections_for_confirmation').value
        
        # Publishing
        self.publish_markers = self.get_parameter('publish_markers').value
        self.publish_debug_images = self.get_parameter('publish_debug_images').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Safety
        self.safety_buffer_distance = self.get_parameter('safety_buffer_distance').value
        self.danger_zone_distance = self.get_parameter('danger_zone_distance').value
    
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
    
    def detect_obstacles(self, depth_image, color_image=None):
        """
        Detect obstacles in depth image
        Returns list of detected obstacles
        """
        if depth_image is None or self.camera_info is None:
            return []
        
        try:
            # Convert depth to meters
            depth_m = depth_image.astype(np.float32) / 1000.0
            
            # Create valid depth mask
            valid_mask = (depth_m > self.min_detection_distance) & (depth_m < self.max_detection_distance)
            
            # Estimate ground plane
            ground_level = self.estimate_ground_plane(depth_m, valid_mask)
            if ground_level is None:
                self.get_logger().debug('Failed to estimate ground plane')
                return []
            
            # Find points above ground (potential obstacles)
            height_above_ground = ground_level - depth_m
            obstacle_mask = (height_above_ground > self.min_obstacle_height) & valid_mask
            
            # Find obstacle clusters
            clusters = self.find_clusters(obstacle_mask, depth_m)
            
            # Convert clusters to obstacle objects
            obstacles = []
            for cluster in clusters:
                obstacle = self.cluster_to_obstacle(cluster, depth_m, color_image)
                if obstacle is not None:
                    obstacles.append(obstacle)
            
            return obstacles
            
        except Exception as e:
            self.get_logger().error(f'Error in obstacle detection: {e}')
            return []
    
    def estimate_ground_plane(self, depth_image, valid_mask):
        """Estimate the ground plane level"""
        try:
            # Sample depth values from bottom portion of image (likely ground)
            h, w = depth_image.shape
            ground_region = depth_image[int(0.6*h):h, :]  # Bottom 40% of image
            ground_valid = valid_mask[int(0.6*h):h, :]
            
            ground_depths = ground_region[ground_valid]
            
            if len(ground_depths) < 100:  # Need sufficient ground samples
                return None
            
            # Use median depth as ground level estimate
            ground_level = np.median(ground_depths)
            
            return ground_level
            
        except Exception as e:
            self.get_logger().debug(f'Ground plane estimation failed: {e}')
            return None
    
    def find_clusters(self, obstacle_mask, depth_image):
        """Find clusters of obstacle points"""
        try:
            # Find connected components in obstacle mask
            num_labels, labels = cv2.connectedComponents(obstacle_mask.astype(np.uint8))
            
            clusters = []
            
            for label in range(1, num_labels):  # Skip background (0)
                cluster_mask = (labels == label)
                cluster_points = np.sum(cluster_mask)
                
                # Filter by cluster size
                if cluster_points < self.min_cluster_size or cluster_points > self.max_cluster_size:
                    continue
                
                # Get cluster coordinates
                y_coords, x_coords = np.where(cluster_mask)
                depths = depth_image[cluster_mask]
                
                # Convert to 3D points using camera intrinsics
                points_3d = []
                for i in range(len(x_coords)):
                    if depths[i] > 0:
                        # Convert pixel coordinates to 3D using camera intrinsics
                        x_3d = (x_coords[i] - self.camera_info.k[2]) * depths[i] / self.camera_info.k[0]
                        y_3d = (y_coords[i] - self.camera_info.k[5]) * depths[i] / self.camera_info.k[4]
                        z_3d = depths[i]
                        points_3d.append([x_3d, y_3d, z_3d])
                
                if len(points_3d) > self.min_cluster_size:
                    clusters.append({
                        'points_3d': np.array(points_3d),
                        'mask': cluster_mask,
                        'pixel_coords': np.column_stack((x_coords, y_coords))
                    })
            
            return clusters
            
        except Exception as e:
            self.get_logger().debug(f'Clustering failed: {e}')
            return []
    
    def cluster_to_obstacle(self, cluster, depth_image, color_image):
        """Convert point cluster to obstacle object"""
        try:
            points = cluster['points_3d']
            if len(points) == 0:
                return None
            
            # Calculate obstacle properties
            min_point = np.min(points, axis=0)
            max_point = np.max(points, axis=0)
            center = np.mean(points, axis=0)
            
            # Size calculation
            size = max_point - min_point
            width, depth, height = size[0], size[2], size[1]  # x, z, y
            
            # Create obstacle object
            obstacle = {
                'position': center,
                'size': [width, depth, height],
                'height': height,
                'points': points,
                'pixel_coords': cluster['pixel_coords'],
                'confidence': self.calculate_obstacle_confidence(cluster, points),
                'type': self.classify_obstacle(size, points),
                'footprint': self.calculate_footprint(points),
                'danger_level': self.calculate_danger_level(center, size),
                'is_static': True,  # Assume static for now
                'is_temporary': False
            }
            
            return obstacle
            
        except Exception as e:
            self.get_logger().debug(f'Failed to convert cluster to obstacle: {e}')
            return None
    
    def calculate_obstacle_confidence(self, cluster, points):
        """Calculate confidence in obstacle detection"""
        try:
            # Factors for confidence calculation
            factors = []
            
            # Point density factor
            point_density = len(points) / np.prod(np.max(points, axis=0) - np.min(points, axis=0))
            factors.append(min(point_density / 1000, 1.0))  # Normalize
            
            # Size consistency factor (how well-formed the obstacle is)
            size = np.max(points, axis=0) - np.min(points, axis=0)
            aspect_ratios = [size[1]/size[0], size[2]/size[0], size[2]/size[1]]  # y/x, z/x, z/y
            consistency = 1.0 - np.std(aspect_ratios) / np.mean(aspect_ratios)
            factors.append(max(consistency, 0.0))
            
            # Distance factor (closer obstacles are more confident)
            distance = np.linalg.norm(np.mean(points, axis=0))
            distance_factor = max(1.0 - distance / self.max_detection_distance, 0.0)
            factors.append(distance_factor)
            
            return float(np.mean(factors))
            
        except Exception as e:
            return 0.5  # Default confidence
    
    def classify_obstacle(self, size, points):
        """Classify obstacle type based on size and shape"""
        try:
            width, depth, height = size[0], size[2], size[1]
            
            # Tree detection (tall and narrow)
            if height > self.tree_height_threshold:
                aspect_ratio = height / max(width, depth)
                if aspect_ratio > 2.0:
                    return 'tree'
            
            # Rock detection (roughly spherical/cubic)
            if height > 0.1 and height < 1.0:
                aspect_ratio = min(width, depth) / max(width, depth)
                if aspect_ratio > self.rock_aspect_ratio_threshold:
                    return 'rock'
            
            # Furniture detection (medium size, rectangular)
            if (width > self.furniture_size_threshold or 
                depth > self.furniture_size_threshold) and height > 0.3:
                return 'furniture'
            
            # Small obstacles
            if height < 0.3:
                return 'small_obstacle'
            
            # Default classification
            return 'unknown'
            
        except Exception as e:
            return 'unknown'
    
    def calculate_footprint(self, points):
        """Calculate 2D footprint polygon of obstacle"""
        try:
            if len(points) < 3:
                return None
            
            # Project points to 2D (x-z plane)
            points_2d = points[:, [0, 2]]  # x and z coordinates
            
            # Find convex hull
            from scipy.spatial import ConvexHull
            hull = ConvexHull(points_2d)
            
            # Create polygon
            polygon = Polygon()
            for vertex_idx in hull.vertices:
                point = Point32()
                point.x = float(points_2d[vertex_idx, 0])
                point.y = float(points_2d[vertex_idx, 1])
                point.z = 0.0
                polygon.points.append(point)
            
            return polygon
            
        except Exception as e:
            return None
    
    def calculate_danger_level(self, position, size):
        """Calculate danger level based on position and size"""
        try:
            distance = np.linalg.norm(position)
            max_dimension = max(size)
            
            # Higher danger for closer and larger obstacles
            distance_factor = max(0.0, 1.0 - distance / self.danger_zone_distance)
            size_factor = min(max_dimension / 2.0, 1.0)
            
            danger_level = (distance_factor + size_factor) / 2.0
            return float(min(danger_level, 1.0))
            
        except Exception as e:
            return 0.5
    
    def track_obstacles(self, detected_obstacles):
        """Track obstacles over time and assign persistent IDs"""
        current_time = time.time()
        
        # Match detected obstacles to tracked obstacles
        matched_obstacles = []
        unmatched_detections = list(detected_obstacles)
        
        for obstacle_id, tracked in list(self.tracked_obstacles.items()):
            # Check if tracked obstacle has timed out
            if current_time - tracked['last_seen'] > self.obstacle_timeout:
                del self.tracked_obstacles[obstacle_id]
                continue
            
            # Find best matching detection
            best_match = None
            best_distance = float('inf')
            
            for detection in unmatched_detections:
                distance = np.linalg.norm(np.array(detection['position']) - np.array(tracked['position']))
                if distance < self.position_tolerance and distance < best_distance:
                    best_match = detection
                    best_distance = distance
            
            if best_match is not None:
                # Update tracked obstacle
                tracked['position'] = best_match['position']
                tracked['size'] = best_match['size']
                tracked['height'] = best_match['height']
                tracked['confidence'] = best_match['confidence']
                tracked['type'] = best_match['type']
                tracked['footprint'] = best_match['footprint']
                tracked['danger_level'] = best_match['danger_level']
                tracked['last_seen'] = current_time
                tracked['detection_count'] += 1
                
                matched_obstacles.append((obstacle_id, tracked))
                unmatched_detections.remove(best_match)
        
        # Create new tracked obstacles for unmatched detections
        for detection in unmatched_detections:
            obstacle_id = self.next_obstacle_id
            self.next_obstacle_id += 1
            
            tracked_obstacle = {
                'id': obstacle_id,
                'position': detection['position'],
                'size': detection['size'],
                'height': detection['height'],
                'confidence': detection['confidence'],
                'type': detection['type'],
                'footprint': detection['footprint'],
                'danger_level': detection['danger_level'],
                'first_seen': current_time,
                'last_seen': current_time,
                'detection_count': 1,
                'is_static': detection['is_static'],
                'is_temporary': detection['is_temporary']
            }
            
            self.tracked_obstacles[obstacle_id] = tracked_obstacle
            matched_obstacles.append((obstacle_id, tracked_obstacle))
        
        return matched_obstacles
    
    def create_obstacle_messages(self, tracked_obstacles):
        """Create ROS messages for obstacles"""
        obstacle_array = ObstacleArray()
        obstacle_array.header = Header()
        obstacle_array.header.stamp = self.get_clock().now().to_msg()
        obstacle_array.header.frame_id = self.frame_id
        
        confirmed_obstacles = []
        
        for obstacle_id, tracked in tracked_obstacles:
            # Only publish confirmed obstacles
            if tracked['detection_count'] >= self.min_detections_for_confirmation:
                obstacle_msg = Obstacle()
                obstacle_msg.id = obstacle_id
                obstacle_msg.type = tracked['type']
                
                # Position
                obstacle_msg.position = Point()
                obstacle_msg.position.x = float(tracked['position'][0])
                obstacle_msg.position.y = float(tracked['position'][1])
                obstacle_msg.position.z = float(tracked['position'][2])
                
                # Size
                obstacle_msg.size = Vector3()
                obstacle_msg.size.x = float(tracked['size'][0])
                obstacle_msg.size.y = float(tracked['size'][1])
                obstacle_msg.size.z = float(tracked['size'][2])
                
                obstacle_msg.height = float(tracked['height'])
                obstacle_msg.confidence = float(tracked['confidence'])
                obstacle_msg.danger_level = float(tracked['danger_level'])
                obstacle_msg.is_static = tracked['is_static']
                obstacle_msg.is_temporary = tracked['is_temporary']
                obstacle_msg.detection_count = tracked['detection_count']
                
                # Timestamps
                obstacle_msg.first_seen.sec = int(tracked['first_seen'])
                obstacle_msg.first_seen.nanosec = int((tracked['first_seen'] % 1) * 1e9)
                obstacle_msg.last_seen.sec = int(tracked['last_seen'])
                obstacle_msg.last_seen.nanosec = int((tracked['last_seen'] % 1) * 1e9)
                
                # Footprint
                if tracked['footprint'] is not None:
                    obstacle_msg.footprint = tracked['footprint']
                
                # Safety
                obstacle_msg.min_safe_distance = self.safety_buffer_distance
                
                confirmed_obstacles.append(obstacle_msg)
        
        obstacle_array.obstacles = confirmed_obstacles
        obstacle_array.total_obstacles = len(confirmed_obstacles)
        obstacle_array.static_obstacles = sum(1 for obs in confirmed_obstacles if obs.is_static)
        obstacle_array.dynamic_obstacles = sum(1 for obs in confirmed_obstacles if not obs.is_static)
        obstacle_array.temporary_obstacles = sum(1 for obs in confirmed_obstacles if obs.is_temporary)
        obstacle_array.detection_range = self.max_detection_distance
        
        return obstacle_array
    
    def create_visualization_markers(self, obstacle_array):
        """Create visualization markers for RViz"""
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(obstacle_array.obstacles):
            # Create bounding box marker
            marker = Marker()
            marker.header = obstacle_array.header
            marker.id = obstacle.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position = obstacle.position
            marker.pose.orientation.w = 1.0
            
            marker.scale = obstacle.size
            
            # Color based on obstacle type and danger level
            marker.color = self.get_obstacle_color(obstacle.type, obstacle.danger_level)
            marker.color.a = 0.7
            
            marker.lifetime.sec = int(self.obstacle_timeout)
            marker_array.markers.append(marker)
            
            # Create text label
            text_marker = Marker()
            text_marker.header = obstacle_array.header
            text_marker.id = obstacle.id + 10000  # Offset to avoid ID conflicts
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position = obstacle.position
            text_marker.pose.position.z += obstacle.size.z / 2 + 0.1  # Above obstacle
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{obstacle.type}\n{obstacle.confidence:.2f}"
            text_marker.lifetime.sec = int(self.obstacle_timeout)
            marker_array.markers.append(text_marker)
        
        return marker_array
    
    def get_obstacle_color(self, obstacle_type, danger_level):
        """Get color for obstacle visualization"""
        color = ColorRGBA()
        
        # Base color by type
        if obstacle_type == 'tree':
            color.r, color.g, color.b = 0.4, 0.8, 0.2  # Green
        elif obstacle_type == 'rock':
            color.r, color.g, color.b = 0.6, 0.6, 0.6  # Gray
        elif obstacle_type == 'furniture':
            color.r, color.g, color.b = 0.8, 0.6, 0.4  # Brown
        elif obstacle_type == 'small_obstacle':
            color.r, color.g, color.b = 0.8, 0.8, 0.2  # Yellow
        else:
            color.r, color.g, color.b = 0.8, 0.4, 0.8  # Purple
        
        # Modify intensity based on danger level
        intensity = 0.5 + 0.5 * danger_level
        color.r *= intensity
        color.g *= intensity
        color.b *= intensity
        
        return color
    
    def create_debug_image(self, color_image, obstacles):
        """Create debug visualization image"""
        if color_image is None:
            return None
        
        debug_image = color_image.copy()
        
        # Draw obstacle bounding boxes
        for obstacle in obstacles:
            if 'pixel_coords' in obstacle:
                # Draw obstacle points
                for coord in obstacle['pixel_coords']:
                    cv2.circle(debug_image, (coord[0], coord[1]), 2, (0, 255, 0), -1)
                
                # Draw bounding rectangle
                x_coords = obstacle['pixel_coords'][:, 0]
                y_coords = obstacle['pixel_coords'][:, 1]
                x_min, x_max = np.min(x_coords), np.max(x_coords)
                y_min, y_max = np.min(y_coords), np.max(y_coords)
                
                cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                
                # Add obstacle info
                label = f"{obstacle['type']} ({obstacle['confidence']:.2f})"
                cv2.putText(debug_image, label, (x_min, y_min - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return debug_image
    
    def detection_callback(self):
        """Main detection callback"""
        if not self.detection_active:
            return
        
        try:
            # Detect obstacles
            detected_obstacles = self.detect_obstacles(self.latest_depth_image, self.latest_color_image)
            
            # Track obstacles over time
            tracked_obstacles = self.track_obstacles(detected_obstacles)
            
            # Create and publish obstacle array message
            obstacle_array = self.create_obstacle_messages(tracked_obstacles)
            self.obstacle_array_pub.publish(obstacle_array)
            
            # Publish visualization markers
            if self.publish_markers:
                markers = self.create_visualization_markers(obstacle_array)
                self.marker_pub.publish(markers)
            
            # Publish debug image
            if self.publish_debug_images and self.latest_color_image is not None:
                debug_image = self.create_debug_image(self.latest_color_image, detected_obstacles)
                if debug_image is not None:
                    try:
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                        debug_msg.header = obstacle_array.header
                        self.debug_image_pub.publish(debug_msg)
                    except Exception as e:
                        self.get_logger().debug(f'Failed to publish debug image: {e}')
            
            # Log detection summary
            if len(obstacle_array.obstacles) > 0:
                self.get_logger().info(f'Detected {len(obstacle_array.obstacles)} obstacles: '
                                     f'{obstacle_array.static_obstacles} static, '
                                     f'{obstacle_array.dynamic_obstacles} dynamic')
            
        except Exception as e:
            self.get_logger().error(f'Error in obstacle detection callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        obstacle_detector = ObstacleDetector()
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