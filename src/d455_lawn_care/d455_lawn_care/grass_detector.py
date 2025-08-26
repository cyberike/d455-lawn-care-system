#!/usr/bin/env python3
"""
Grass Detection Node for D455 Lawn Care
Real-time grass detection and analysis using RGB and depth data from Intel RealSense D455
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
import yaml
import os

# ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Header
from d455_lawn_care.msg import GrassDetection
from d455_lawn_care.srv import CalibrateGrassDetection

class GrassDetector(Node):
    def __init__(self):
        super().__init__('grass_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # QoS profile for image topics (best effort for real-time performance)
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Declare parameters with defaults optimized for outdoor grass detection
        self.declare_parameters(
            namespace='',
            parameters=[
                # HSV color range for grass detection
                ('grass_color_hue_min', 40),        # Lower green hue
                ('grass_color_hue_max', 80),        # Upper green hue  
                ('grass_color_sat_min', 40),        # Minimum saturation
                ('grass_color_sat_max', 255),       # Maximum saturation
                ('grass_color_val_min', 40),        # Minimum brightness
                ('grass_color_val_max', 255),       # Maximum brightness
                
                # Grass height analysis
                ('min_grass_height', 0.02),         # 2cm minimum height
                ('max_grass_height', 0.15),         # 15cm maximum height
                ('cutting_threshold', 0.08),        # 8cm cutting threshold
                
                # Detection parameters
                ('grass_coverage_threshold', 0.3),   # 30% minimum grass coverage
                ('min_grass_area', 1000),           # Minimum grass area in pixels
                ('gaussian_blur_kernel', 5),        # Blur kernel size
                ('morphology_kernel_size', 3),      # Morphological operation kernel
                
                # Publishing settings
                ('publish_debug_images', True),     # Whether to publish debug images
                ('detection_frequency', 10.0),     # Detection frequency in Hz
                ('frame_id', 'd455_color_optical_frame'),  # Frame ID for detections
            ]
        )
        
        # Get parameters
        self.update_parameters()
        
        # Initialize variables
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_info = None
        self.detection_active = True
        
        # Create subscribers
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
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/d455/d455_camera/color/camera_info', 
            self.camera_info_callback,
            10
        )
        
        # Create publishers
        self.grass_detection_pub = self.create_publisher(
            GrassDetection,
            '~/grass_detection',
            10
        )
        
        if self.publish_debug_images:
            self.debug_image_pub = self.create_publisher(
                Image,
                '~/debug_image',
                10
            )
            
            self.mask_image_pub = self.create_publisher(
                Image,
                '~/grass_mask',
                10
            )
        
        # Create service for calibration
        self.calibration_service = self.create_service(
            CalibrateGrassDetection,
            '~/calibrate_grass_detection',
            self.calibrate_grass_detection_callback
        )
        
        # Create detection timer
        self.detection_timer = self.create_timer(
            1.0 / self.detection_frequency,
            self.detection_callback
        )
        
        self.get_logger().info('Grass Detector Node initialized')
        self.get_logger().info(f'HSV Range: H({self.hue_min}-{self.hue_max}), '
                              f'S({self.sat_min}-{self.sat_max}), '
                              f'V({self.val_min}-{self.val_max})')
    
    def update_parameters(self):
        """Update parameters from ROS parameter server"""
        # HSV color range
        self.hue_min = self.get_parameter('grass_color_hue_min').value
        self.hue_max = self.get_parameter('grass_color_hue_max').value
        self.sat_min = self.get_parameter('grass_color_sat_min').value
        self.sat_max = self.get_parameter('grass_color_sat_max').value
        self.val_min = self.get_parameter('grass_color_val_min').value
        self.val_max = self.get_parameter('grass_color_val_max').value
        
        # Grass height parameters
        self.min_grass_height = self.get_parameter('min_grass_height').value
        self.max_grass_height = self.get_parameter('max_grass_height').value
        self.cutting_threshold = self.get_parameter('cutting_threshold').value
        
        # Detection parameters
        self.grass_coverage_threshold = self.get_parameter('grass_coverage_threshold').value
        self.min_grass_area = self.get_parameter('min_grass_area').value
        self.gaussian_blur_kernel = self.get_parameter('gaussian_blur_kernel').value
        self.morphology_kernel_size = self.get_parameter('morphology_kernel_size').value
        
        # Publishing settings
        self.publish_debug_images = self.get_parameter('publish_debug_images').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Create HSV bounds
        self.lower_grass = np.array([self.hue_min, self.sat_min, self.val_min])
        self.upper_grass = np.array([self.hue_max, self.sat_max, self.val_max])
    
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
    
    def camera_info_callback(self, msg):
        """Callback for camera info"""
        self.camera_info = msg
    
    def detect_grass(self, color_image, depth_image):
        """
        Detect grass in color and depth images
        Returns grass detection results
        """
        if color_image is None:
            return None
            
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        if self.gaussian_blur_kernel > 1:
            hsv = cv2.GaussianBlur(hsv, (self.gaussian_blur_kernel, self.gaussian_blur_kernel), 0)
        
        # Create grass mask using HSV color range
        grass_mask = cv2.inRange(hsv, self.lower_grass, self.upper_grass)
        
        # Apply morphological operations to clean up mask
        kernel = np.ones((self.morphology_kernel_size, self.morphology_kernel_size), np.uint8)
        grass_mask = cv2.morphologyEx(grass_mask, cv2.MORPH_CLOSE, kernel)
        grass_mask = cv2.morphologyEx(grass_mask, cv2.MORPH_OPEN, kernel)
        
        # Calculate grass coverage
        total_pixels = grass_mask.shape[0] * grass_mask.shape[1]
        grass_pixels = np.sum(grass_mask > 0)
        grass_coverage = grass_pixels / total_pixels
        
        # Initialize results
        results = {
            'grass_coverage_percentage': grass_coverage * 100,
            'grass_pixel_count': int(grass_pixels),
            'total_pixel_count': int(total_pixels),
            'detection_confidence': 0.0,
            'average_grass_height': 0.0,
            'min_grass_height': 0.0,
            'max_grass_height': 0.0,
            'grass_density': 0.0,
            'grass_greenness': 0.0,
            'grass_needs_cutting': False,
            'grass_regions': [],
            'grass_mask': grass_mask,
            'debug_image': color_image.copy()
        }
        
        # Only proceed with detailed analysis if we have sufficient grass coverage
        if grass_coverage < self.grass_coverage_threshold / 100:
            results['detection_confidence'] = 0.3  # Low confidence due to insufficient coverage
            return results
        
        # Analyze grass height using depth data if available
        if depth_image is not None:
            grass_heights = self.analyze_grass_height(grass_mask, depth_image)
            if grass_heights is not None and len(grass_heights) > 0:
                results['average_grass_height'] = np.mean(grass_heights)
                results['min_grass_height'] = np.min(grass_heights)
                results['max_grass_height'] = np.max(grass_heights)
                results['grass_needs_cutting'] = results['average_grass_height'] > self.cutting_threshold
        
        # Calculate grass density (how densely packed the grass pixels are)
        results['grass_density'] = self.calculate_grass_density(grass_mask)
        
        # Calculate grass greenness (how green the detected grass appears)
        results['grass_greenness'] = self.calculate_grass_greenness(hsv, grass_mask)
        
        # Find grass regions (contours)
        results['grass_regions'] = self.find_grass_regions(grass_mask)
        
        # Set detection confidence based on various factors
        results['detection_confidence'] = self.calculate_confidence(results)
        
        # Create debug image
        if self.publish_debug_images:
            results['debug_image'] = self.create_debug_image(color_image, grass_mask, results)
        
        return results
    
    def analyze_grass_height(self, grass_mask, depth_image):
        """Analyze grass height using depth information"""
        try:
            # Get depth values only for grass pixels
            grass_depths = depth_image[grass_mask > 0]
            
            # Convert from mm to meters (assuming depth is in mm)
            grass_depths = grass_depths.astype(np.float32) / 1000.0
            
            # Filter out invalid depths (0 or very large values)
            valid_depths = grass_depths[(grass_depths > 0.1) & (grass_depths < 10.0)]
            
            if len(valid_depths) == 0:
                return None
            
            # Estimate ground plane (assume most common depth is ground level)
            ground_level = np.percentile(valid_depths, 10)  # Bottom 10% as ground estimate
            
            # Calculate grass heights relative to ground
            grass_heights = ground_level - valid_depths
            
            # Filter reasonable grass heights
            grass_heights = grass_heights[(grass_heights >= 0) & (grass_heights <= 0.5)]
            
            return grass_heights
            
        except Exception as e:
            self.get_logger().debug(f'Failed to analyze grass height: {e}')
            return None
    
    def calculate_grass_density(self, grass_mask):
        """Calculate how densely packed the grass pixels are"""
        try:
            # Find contours in the grass mask
            contours, _ = cv2.findContours(grass_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return 0.0
            
            total_area = 0
            total_perimeter = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                
                if area > self.min_grass_area:  # Only consider significant areas
                    total_area += area
                    total_perimeter += perimeter
            
            if total_perimeter == 0:
                return 0.0
            
            # Density metric: higher values indicate more compact grass regions
            density = (4 * np.pi * total_area) / (total_perimeter ** 2)
            return min(density, 1.0)  # Clamp to [0, 1]
            
        except Exception as e:
            self.get_logger().debug(f'Failed to calculate grass density: {e}')
            return 0.0
    
    def calculate_grass_greenness(self, hsv_image, grass_mask):
        """Calculate how green the detected grass appears"""
        try:
            # Extract HSV values for grass pixels
            grass_hsv = hsv_image[grass_mask > 0]
            
            if len(grass_hsv) == 0:
                return 0.0
            
            # Calculate greenness based on saturation and value
            saturation = grass_hsv[:, 1].astype(np.float32) / 255.0
            value = grass_hsv[:, 2].astype(np.float32) / 255.0
            
            # Greenness is combination of saturation and brightness
            greenness = np.mean(saturation * value)
            
            return float(greenness)
            
        except Exception as e:
            self.get_logger().debug(f'Failed to calculate grass greenness: {e}')
            return 0.0
    
    def find_grass_regions(self, grass_mask):
        """Find grass regions as polygons"""
        try:
            polygons = []
            
            # Find contours
            contours, _ = cv2.findContours(grass_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_grass_area:
                    # Simplify contour to polygon
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    
                    # Convert to ROS Polygon message
                    polygon = Polygon()
                    for point in approx:
                        pt = Point32()
                        pt.x = float(point[0][0])
                        pt.y = float(point[0][1])
                        pt.z = 0.0
                        polygon.points.append(pt)
                    
                    polygons.append(polygon)
            
            return polygons
            
        except Exception as e:
            self.get_logger().debug(f'Failed to find grass regions: {e}')
            return []
    
    def calculate_confidence(self, results):
        """Calculate overall detection confidence"""
        confidence_factors = []
        
        # Coverage factor
        coverage_factor = min(results['grass_coverage_percentage'] / 50.0, 1.0)
        confidence_factors.append(coverage_factor)
        
        # Density factor
        confidence_factors.append(results['grass_density'])
        
        # Greenness factor
        confidence_factors.append(results['grass_greenness'])
        
        # Pixel count factor
        pixel_factor = min(results['grass_pixel_count'] / 10000, 1.0)
        confidence_factors.append(pixel_factor)
        
        # Calculate weighted average
        return float(np.mean(confidence_factors))
    
    def create_debug_image(self, color_image, grass_mask, results):
        """Create debug visualization image"""
        debug_image = color_image.copy()
        
        # Apply grass mask overlay
        grass_overlay = np.zeros_like(debug_image)
        grass_overlay[grass_mask > 0] = [0, 255, 0]  # Green overlay
        debug_image = cv2.addWeighted(debug_image, 0.7, grass_overlay, 0.3, 0)
        
        # Draw grass regions
        for polygon in results['grass_regions']:
            points = np.array([[int(pt.x), int(pt.y)] for pt in polygon.points])
            if len(points) > 2:
                cv2.polylines(debug_image, [points], True, (255, 0, 0), 2)
        
        # Add text information
        info_text = [
            f"Coverage: {results['grass_coverage_percentage']:.1f}%",
            f"Confidence: {results['detection_confidence']:.2f}",
            f"Height: {results['average_grass_height']:.3f}m",
            f"Density: {results['grass_density']:.2f}",
            f"Greenness: {results['grass_greenness']:.2f}"
        ]
        
        y_offset = 30
        for text in info_text:
            cv2.putText(debug_image, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_offset += 25
        
        return debug_image
    
    def detection_callback(self):
        """Main detection callback executed at regular intervals"""
        if not self.detection_active or self.latest_color_image is None:
            return
        
        try:
            # Perform grass detection
            results = self.detect_grass(self.latest_color_image, self.latest_depth_image)
            
            if results is None:
                return
            
            # Create and publish grass detection message
            grass_msg = GrassDetection()
            grass_msg.header = Header()
            grass_msg.header.stamp = self.get_clock().now().to_msg()
            grass_msg.header.frame_id = self.frame_id
            
            # Fill in detection results
            grass_msg.grass_coverage_percentage = results['grass_coverage_percentage']
            grass_msg.average_grass_height = results['average_grass_height']
            grass_msg.min_grass_height = results['min_grass_height']
            grass_msg.max_grass_height = results['max_grass_height']
            grass_msg.grass_density = results['grass_density']
            grass_msg.grass_greenness = results['grass_greenness']
            grass_msg.grass_needs_cutting = results['grass_needs_cutting']
            grass_msg.detection_confidence = results['detection_confidence']
            grass_msg.grass_pixel_count = results['grass_pixel_count']
            grass_msg.total_pixel_count = results['total_pixel_count']
            grass_msg.grass_regions = results['grass_regions']
            
            # Publish grass detection
            self.grass_detection_pub.publish(grass_msg)
            
            # Publish debug images if enabled
            if self.publish_debug_images:
                try:
                    debug_img_msg = self.bridge.cv2_to_imgmsg(results['debug_image'], "bgr8")
                    debug_img_msg.header = grass_msg.header
                    self.debug_image_pub.publish(debug_img_msg)
                    
                    mask_img_msg = self.bridge.cv2_to_imgmsg(results['grass_mask'], "mono8")
                    mask_img_msg.header = grass_msg.header
                    self.mask_image_pub.publish(mask_img_msg)
                    
                except Exception as e:
                    self.get_logger().debug(f'Failed to publish debug images: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Error in grass detection: {e}')
    
    def calibrate_grass_detection_callback(self, request, response):
        """Service callback for grass detection calibration"""
        try:
            self.get_logger().info('Starting grass detection calibration...')
            
            # Use current image if no sample provided
            if request.sample_image.data:
                calibration_image = self.bridge.imgmsg_to_cv2(request.sample_image, "bgr8")
            else:
                calibration_image = self.latest_color_image
            
            if calibration_image is None:
                response.success = False
                response.message = "No image available for calibration"
                return response
            
            if request.use_manual_params:
                # Use provided manual parameters
                self.hue_min = request.hue_min
                self.hue_max = request.hue_max
                self.sat_min = request.saturation_min
                self.sat_max = request.saturation_max
                self.val_min = request.value_min
                self.val_max = request.value_max
                
                self.get_logger().info('Using manual calibration parameters')
            else:
                # Auto-calibrate based on image content
                calibrated_params = self.auto_calibrate_hsv(calibration_image)
                if calibrated_params is None:
                    response.success = False
                    response.message = "Auto-calibration failed"
                    return response
                
                self.hue_min, self.hue_max = calibrated_params['hue']
                self.sat_min, self.sat_max = calibrated_params['saturation'] 
                self.val_min, self.val_max = calibrated_params['value']
            
            # Update HSV bounds
            self.lower_grass = np.array([self.hue_min, self.sat_min, self.val_min])
            self.upper_grass = np.array([self.hue_max, self.sat_max, self.val_max])
            
            # Test calibration on current image
            test_results = self.detect_grass(calibration_image, self.latest_depth_image)
            
            # Fill response
            response.success = True
            response.message = f"Calibration successful. Coverage: {test_results['grass_coverage_percentage']:.1f}%"
            
            response.calibrated_hue_min = self.hue_min
            response.calibrated_hue_max = self.hue_max
            response.calibrated_saturation_min = self.sat_min
            response.calibrated_saturation_max = self.sat_max
            response.calibrated_value_min = self.val_min
            response.calibrated_value_max = self.val_max
            
            response.detection_accuracy = test_results['detection_confidence']
            response.coverage_percentage = test_results['grass_coverage_percentage']
            
            # Create debug image
            if test_results and 'debug_image' in test_results:
                response.debug_image = self.bridge.cv2_to_imgmsg(test_results['debug_image'], "bgr8")
            
            self.get_logger().info(f'Calibration completed: {response.message}')
            
        except Exception as e:
            response.success = False
            response.message = f"Calibration error: {str(e)}"
            self.get_logger().error(f'Calibration failed: {e}')
        
        return response
    
    def auto_calibrate_hsv(self, image):
        """Automatically calibrate HSV parameters based on image content"""
        try:
            # Convert to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Focus on center region for grass sampling
            h, w = hsv.shape[:2]
            center_region = hsv[h//4:3*h//4, w//4:3*w//4]
            
            # Sample green pixels (rough initial filtering)
            rough_mask = cv2.inRange(center_region, 
                                   np.array([30, 30, 30]), 
                                   np.array([90, 255, 255]))
            
            # Get HSV values of detected green pixels
            green_pixels = center_region[rough_mask > 0]
            
            if len(green_pixels) < 100:  # Need sufficient samples
                return None
            
            # Calculate statistics for each channel
            hue_values = green_pixels[:, 0]
            sat_values = green_pixels[:, 1] 
            val_values = green_pixels[:, 2]
            
            # Use percentiles to avoid outliers
            hue_range = (int(np.percentile(hue_values, 5)), int(np.percentile(hue_values, 95)))
            sat_range = (int(np.percentile(sat_values, 10)), int(np.percentile(sat_values, 90)))
            val_range = (int(np.percentile(val_values, 10)), int(np.percentile(val_values, 90)))
            
            return {
                'hue': hue_range,
                'saturation': sat_range, 
                'value': val_range
            }
            
        except Exception as e:
            self.get_logger().error(f'Auto-calibration failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    try:
        grass_detector = GrassDetector()
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