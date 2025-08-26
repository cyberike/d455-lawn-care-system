#!/usr/bin/env python3
"""
Simplified Grass Detection Node - Using Standard Messages
Real-time grass detection using RGB and depth data from D455
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

# Standard ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Header, String
from geometry_msgs.msg import PointStamped

class SimpleGrassDetector(Node):
    def __init__(self):
        super().__init__('simple_grass_detector')
        
        self.bridge = CvBridge()
        
        # QoS profile for image topics
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Declare and get grass detection parameters
        self.declare_parameter('simple_grass_detector.hue_min', 40)
        self.declare_parameter('simple_grass_detector.hue_max', 80)
        self.declare_parameter('simple_grass_detector.sat_min', 40)
        self.declare_parameter('simple_grass_detector.sat_max', 255)
        self.declare_parameter('simple_grass_detector.val_min', 40)
        self.declare_parameter('simple_grass_detector.val_max', 255)
        self.declare_parameter('simple_grass_detector.gaussian_blur_kernel', 5)
        self.declare_parameter('simple_grass_detector.morphology_kernel', 3)
        self.declare_parameter('simple_grass_detector.detection_rate', 10.0)
        self.declare_parameter('simple_grass_detector.coverage_threshold', 30.0)
        self.declare_parameter('simple_grass_detector.log_threshold', 10.0)
        
        # Get parameter values
        self.hue_min = self.get_parameter('simple_grass_detector.hue_min').get_parameter_value().integer_value
        self.hue_max = self.get_parameter('simple_grass_detector.hue_max').get_parameter_value().integer_value
        self.sat_min = self.get_parameter('simple_grass_detector.sat_min').get_parameter_value().integer_value
        self.sat_max = self.get_parameter('simple_grass_detector.sat_max').get_parameter_value().integer_value
        self.val_min = self.get_parameter('simple_grass_detector.val_min').get_parameter_value().integer_value
        self.val_max = self.get_parameter('simple_grass_detector.val_max').get_parameter_value().integer_value
        self.blur_kernel = self.get_parameter('simple_grass_detector.gaussian_blur_kernel').get_parameter_value().integer_value
        self.morph_kernel = self.get_parameter('simple_grass_detector.morphology_kernel').get_parameter_value().integer_value
        self.detection_rate = self.get_parameter('simple_grass_detector.detection_rate').get_parameter_value().double_value
        self.coverage_threshold = self.get_parameter('simple_grass_detector.coverage_threshold').get_parameter_value().double_value
        self.log_threshold = self.get_parameter('simple_grass_detector.log_threshold').get_parameter_value().double_value
        
        self.lower_grass = np.array([self.hue_min, self.sat_min, self.val_min])
        self.upper_grass = np.array([self.hue_max, self.sat_max, self.val_max])
        
        # Initialize variables
        self.latest_color_image = None
        self.latest_depth_image = None
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image, 
            '/d455/d455_camera/color/image_raw',
            self.color_callback,
            self.image_qos
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            self.image_qos
        )
        
        # Publishers using standard messages
        self.grass_coverage_pub = self.create_publisher(
            Float32,
            '~/grass_coverage_percentage',
            10
        )
        
        self.grass_status_pub = self.create_publisher(
            String,
            '~/grass_status',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '~/debug_image',
            10
        )
        
        self.grass_mask_pub = self.create_publisher(
            Image,
            '~/grass_mask',
            10
        )
        
        # Detection timer with configurable rate
        timer_period = 1.0 / self.detection_rate
        self.detection_timer = self.create_timer(timer_period, self.detection_callback)
        
        self.get_logger().info('Simple Grass Detector Node initialized')
        self.get_logger().info(f'HSV Range: H({self.hue_min}-{self.hue_max}), '
                              f'S({self.sat_min}-{self.sat_max}), '
                              f'V({self.val_min}-{self.val_max})')
    
    def color_callback(self, msg):
        """Callback for color image"""
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
    
    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def detect_grass(self, color_image):
        """Detect grass in color image"""
        if color_image is None:
            return None
            
        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur with configurable kernel
        kernel_size = (self.blur_kernel, self.blur_kernel)
        hsv = cv2.GaussianBlur(hsv, kernel_size, 0)
        
        # Create grass mask
        grass_mask = cv2.inRange(hsv, self.lower_grass, self.upper_grass)
        
        # Clean up mask with configurable morphological operations
        kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        grass_mask = cv2.morphologyEx(grass_mask, cv2.MORPH_CLOSE, kernel)
        grass_mask = cv2.morphologyEx(grass_mask, cv2.MORPH_OPEN, kernel)
        
        # Calculate coverage
        total_pixels = grass_mask.shape[0] * grass_mask.shape[1]
        grass_pixels = np.sum(grass_mask > 0)
        grass_coverage = (grass_pixels / total_pixels) * 100
        
        # Create debug image
        debug_image = color_image.copy()
        grass_overlay = np.zeros_like(debug_image)
        grass_overlay[grass_mask > 0] = [0, 255, 0]  # Green overlay
        debug_image = cv2.addWeighted(debug_image, 0.7, grass_overlay, 0.3, 0)
        
        # Add text
        cv2.putText(debug_image, f"Grass Coverage: {grass_coverage:.1f}%", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return {
            'coverage': grass_coverage,
            'mask': grass_mask,
            'debug_image': debug_image
        }
    
    def detection_callback(self):
        """Main detection callback"""
        if self.latest_color_image is None:
            return
        
        try:
            # Detect grass
            results = self.detect_grass(self.latest_color_image)
            
            if results is None:
                return
            
            # Publish coverage percentage
            coverage_msg = Float32()
            coverage_msg.data = results['coverage']
            self.grass_coverage_pub.publish(coverage_msg)
            
            # Publish status with configurable threshold
            status_msg = String()
            if results['coverage'] > self.coverage_threshold:
                status_msg.data = f"Good grass detected: {results['coverage']:.1f}%"
            else:
                status_msg.data = f"Low grass coverage: {results['coverage']:.1f}%"
            self.grass_status_pub.publish(status_msg)
            
            # Publish debug images
            try:
                # Debug image with overlay
                debug_img_msg = self.bridge.cv2_to_imgmsg(results['debug_image'], "bgr8")
                debug_img_msg.header.stamp = self.get_clock().now().to_msg()
                debug_img_msg.header.frame_id = "d455_color_optical_frame"
                self.debug_image_pub.publish(debug_img_msg)
                
                # Grass mask
                mask_img_msg = self.bridge.cv2_to_imgmsg(results['mask'], "mono8")
                mask_img_msg.header.stamp = self.get_clock().now().to_msg()
                mask_img_msg.header.frame_id = "d455_color_optical_frame"
                self.grass_mask_pub.publish(mask_img_msg)
                
            except Exception as e:
                self.get_logger().debug(f'Failed to publish debug images: {e}')
            
            # Log detection results with configurable threshold
            if results['coverage'] > self.log_threshold:
                self.get_logger().info(f'Grass detected: {results["coverage"]:.1f}% coverage')
            
        except Exception as e:
            self.get_logger().error(f'Error in grass detection: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        grass_detector = SimpleGrassDetector()
        rclpy.spin(grass_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in grass detector: {e}')
    finally:
        if 'grass_detector' in locals():
            grass_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()