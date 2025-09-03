#!/usr/bin/env python3
"""
Laser Weed Incisor Node for D455 Lawn Care
Precision weed targeting using blue diode laser with safety interlocks
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Tuple

# ROS 2 message types
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, PoseStamped
from std_msgs.msg import Header, Bool, Float32, String
from d455_lawn_care.msg import GrassDetection
from d455_lawn_care.srv import LaserTargeting, LaserCalibration

class LaserState(Enum):
    DISABLED = 0
    STANDBY = 1 
    TARGETING = 2
    FIRING = 3
    EMERGENCY_STOP = 4
    CALIBRATING = 5

@dataclass
class LaserSpecs:
    wavelength_nm: int = 450
    max_power_watts: float = 5.0
    beam_diameter_mm: float = 2.5
    safety_class: str = "Class 4"
    exposure_time_ms: int = 200
    targeting_precision_mm: float = 1.0

@dataclass 
class SafetyConfig:
    enable_emergency_stop: bool = True
    require_safety_confirmation: bool = True
    max_continuous_fire_time_ms: int = 500
    cooldown_time_ms: int = 1000
    safety_zone_radius_m: float = 2.0
    operator_distance_min_m: float = 5.0

class LaserWeedIncisor(Node):
    def __init__(self):
        super().__init__('laser_weed_incisor')
        
        # Laser specifications
        self.laser_specs = LaserSpecs()
        self.safety_config = SafetyConfig()
        
        # System state
        self.current_state = LaserState.DISABLED
        self.laser_enabled = False
        self.emergency_stop_active = False
        self.targeting_queue = []
        self.last_fire_time = 0
        self.total_weeds_targeted = 0
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Laser control parameters
                ('laser_power_percentage', 80.0),      # 80% of max power
                ('exposure_time_ms', 200),             # 200ms exposure per weed
                ('targeting_precision_mm', 1.0),       # 1mm targeting precision
                ('auto_fire_enabled', False),          # Manual fire by default
                
                # Safety parameters
                ('safety_confirmation_required', True), # Require safety confirmation
                ('emergency_stop_enabled', True),      # Enable emergency stop
                ('max_fire_duration_ms', 500),         # Max continuous fire time
                ('cooldown_period_ms', 1000),          # Cooldown between fires
                ('safety_zone_radius_m', 2.0),         # Safety zone around laser
                
                # Detection integration
                ('min_weed_confidence', 0.7),          # Min confidence for targeting
                ('max_weed_size_pixels', 5000),        # Max weed size to target
                ('targeting_frame_id', 'd455_color_optical_frame'),
                
                # Operational parameters
                ('enable_debug_visualization', True),   # Show targeting overlay
                ('log_targeting_stats', True),         # Log targeting statistics
            ]
        )
        
        # Update parameters
        self.update_parameters()
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        self.grass_detection_sub = self.create_subscription(
            GrassDetection,
            '/grass_detector/grass_detection',
            self.grass_detection_callback,
            10
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '~/emergency_stop',
            self.emergency_stop_callback,
            self.reliable_qos
        )
        
        self.safety_enable_sub = self.create_subscription(
            Bool,
            '~/safety_enable',
            self.safety_enable_callback,
            self.reliable_qos
        )
        
        # Create publishers
        self.laser_status_pub = self.create_publisher(
            String,
            '~/laser_status',
            self.reliable_qos
        )
        
        self.targeting_stats_pub = self.create_publisher(
            String,
            '~/targeting_stats', 
            10
        )
        
        if self.enable_debug_visualization:
            self.debug_image_pub = self.create_publisher(
                Image,
                '~/targeting_debug',
                10
            )
        
        # Create services
        self.targeting_service = self.create_service(
            LaserTargeting,
            '~/target_weed',
            self.manual_targeting_callback
        )
        
        self.calibration_service = self.create_service(
            LaserCalibration,
            '~/calibrate_laser',
            self.laser_calibration_callback
        )
        
        # Initialize laser hardware interface (simulated for safety)
        self.laser_interface = self.init_laser_hardware()
        
        # Start safety monitoring thread
        self.safety_monitor_thread = threading.Thread(target=self.safety_monitor_loop, daemon=True)
        self.safety_monitor_thread.start()
        
        # Status update timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Laser Weed Incisor initialized')
        self.get_logger().warning(f'LASER SAFETY: {self.laser_specs.safety_class} laser system')
        self.get_logger().warning('Always wear certified laser safety glasses when operating')
        
    def update_parameters(self):
        """Update parameters from ROS parameter server"""
        self.laser_power_pct = self.get_parameter('laser_power_percentage').value
        self.exposure_time_ms = self.get_parameter('exposure_time_ms').value
        self.targeting_precision_mm = self.get_parameter('targeting_precision_mm').value
        self.auto_fire_enabled = self.get_parameter('auto_fire_enabled').value
        
        self.safety_confirmation_required = self.get_parameter('safety_confirmation_required').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        self.max_fire_duration_ms = self.get_parameter('max_fire_duration_ms').value
        self.cooldown_period_ms = self.get_parameter('cooldown_period_ms').value
        self.safety_zone_radius_m = self.get_parameter('safety_zone_radius_m').value
        
        self.min_weed_confidence = self.get_parameter('min_weed_confidence').value
        self.max_weed_size_pixels = self.get_parameter('max_weed_size_pixels').value
        self.targeting_frame_id = self.get_parameter('targeting_frame_id').value
        
        self.enable_debug_visualization = self.get_parameter('enable_debug_visualization').value
        self.log_targeting_stats = self.get_parameter('log_targeting_stats').value
        
    def init_laser_hardware(self):
        """Initialize laser hardware interface (SIMULATED for safety)"""
        # WARNING: This is a simulated interface for development
        # Real implementation requires proper laser driver integration
        interface = {
            'power_output': 0.0,
            'targeting_servos': {'pan': 0.0, 'tilt': 0.0},
            'safety_interlocks': True,
            'beam_enabled': False
        }
        
        self.get_logger().warn('Using SIMULATED laser interface for safety')
        return interface
        
    def grass_detection_callback(self, msg: GrassDetection):
        """Process grass detection for weed targeting"""
        if self.current_state != LaserState.STANDBY:
            return
            
        if not self.laser_enabled or self.emergency_stop_active:
            return
            
        # Filter detections for weed targeting
        potential_targets = self.identify_weed_targets(msg)
        
        if potential_targets and self.auto_fire_enabled:
            self.queue_weed_targets(potential_targets)
            
    def identify_weed_targets(self, detection: GrassDetection) -> List[Tuple[float, float]]:
        """Identify weed targets from grass detection"""
        targets = []
        
        # Look for grass regions that need cutting (weeds)
        if detection.grass_needs_cutting and detection.detection_confidence >= self.min_weed_confidence:
            # Extract target points from grass regions
            for region in detection.grass_regions:
                if len(region.points) > 0:
                    # Calculate region centroid as target point
                    x_coords = [pt.x for pt in region.points]
                    y_coords = [pt.y for pt in region.points]
                    
                    centroid_x = sum(x_coords) / len(x_coords)
                    centroid_y = sum(y_coords) / len(y_coords)
                    
                    # Calculate region area
                    region_area = self.calculate_polygon_area(region.points)
                    
                    # Only target reasonably sized weeds
                    if region_area <= self.max_weed_size_pixels:
                        targets.append((centroid_x, centroid_y))
                        
        return targets
        
    def calculate_polygon_area(self, points: List[Point32]) -> float:
        """Calculate area of polygon defined by points"""
        if len(points) < 3:
            return 0.0
            
        area = 0.0
        n = len(points)
        
        for i in range(n):
            j = (i + 1) % n
            area += points[i].x * points[j].y
            area -= points[j].x * points[i].y
            
        return abs(area) / 2.0
        
    def queue_weed_targets(self, targets: List[Tuple[float, float]]):
        """Add weed targets to firing queue"""
        for target in targets:
            self.targeting_queue.append({
                'position': target,
                'timestamp': time.time(),
                'priority': 1.0  # Higher values = higher priority
            })
            
        if len(self.targeting_queue) > 0:
            self.current_state = LaserState.TARGETING
            self.process_targeting_queue()
            
    def process_targeting_queue(self):
        """Process queued weed targets"""
        if not self.targeting_queue or self.current_state != LaserState.TARGETING:
            return
            
        # Check cooldown period
        current_time = time.time() * 1000  # Convert to milliseconds
        if (current_time - self.last_fire_time) < self.cooldown_period_ms:
            return
            
        # Get next target
        target = self.targeting_queue.pop(0)
        
        if self.execute_weed_targeting(target):
            self.total_weeds_targeted += 1
            self.last_fire_time = current_time
            
        # Return to standby if queue empty
        if not self.targeting_queue:
            self.current_state = LaserState.STANDBY
            
    def execute_weed_targeting(self, target: dict) -> bool:
        """Execute laser targeting on specific weed"""
        try:
            x, y = target['position']
            
            self.get_logger().info(f'Targeting weed at position ({x:.1f}, {y:.1f})')
            
            # SAFETY CHECK: Verify all safety conditions
            if not self.verify_safety_conditions():
                self.get_logger().error('Safety conditions not met, aborting targeting')
                return False
                
            # Calculate servo positions for targeting
            pan_angle, tilt_angle = self.calculate_servo_angles(x, y)
            
            # Position targeting servos
            if not self.position_targeting_servos(pan_angle, tilt_angle):
                self.get_logger().error('Failed to position targeting servos')
                return False
                
            # Execute laser pulse
            return self.fire_laser_pulse()
            
        except Exception as e:
            self.get_logger().error(f'Error in laser targeting: {e}')
            self.current_state = LaserState.EMERGENCY_STOP
            return False
            
    def verify_safety_conditions(self) -> bool:
        """Verify all safety conditions before firing"""
        if self.emergency_stop_active:
            return False
            
        if not self.laser_enabled:
            return False
            
        if self.safety_confirmation_required and not hasattr(self, '_safety_confirmed'):
            return False
            
        # Check for safety zone clearance (simulated)
        # Real implementation would use proximity sensors
        
        return True
        
    def calculate_servo_angles(self, pixel_x: float, pixel_y: float) -> Tuple[float, float]:
        """Convert pixel coordinates to servo angles"""
        # Simplified conversion - real implementation needs camera calibration
        # Assumes camera field of view and servo range mapping
        
        # Normalize pixel coordinates to [-1, 1] range
        norm_x = (pixel_x - 320) / 320  # Assuming 640px width
        norm_y = (pixel_y - 240) / 240  # Assuming 480px height
        
        # Convert to servo angles (degrees)
        pan_angle = norm_x * 45  # ±45 degree pan range
        tilt_angle = norm_y * 30  # ±30 degree tilt range
        
        return pan_angle, tilt_angle
        
    def position_targeting_servos(self, pan_angle: float, tilt_angle: float) -> bool:
        """Position targeting servos to specified angles"""
        try:
            # SIMULATED: Update servo positions
            self.laser_interface['targeting_servos']['pan'] = pan_angle
            self.laser_interface['targeting_servos']['tilt'] = tilt_angle
            
            # Simulated positioning time
            time.sleep(0.1)
            
            self.get_logger().debug(f'Servos positioned: pan={pan_angle:.1f}°, tilt={tilt_angle:.1f}°')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Servo positioning failed: {e}')
            return False
            
    def fire_laser_pulse(self) -> bool:
        """Execute controlled laser pulse"""
        try:
            self.current_state = LaserState.FIRING
            
            # SAFETY: Final pre-fire check
            if not self.verify_safety_conditions():
                self.current_state = LaserState.STANDBY
                return False
                
            start_time = time.time()
            
            # SIMULATED: Enable laser beam at specified power
            self.laser_interface['power_output'] = (self.laser_power_pct / 100.0) * self.laser_specs.max_power_watts
            self.laser_interface['beam_enabled'] = True
            
            self.get_logger().info(f'LASER FIRING: {self.laser_interface["power_output"]:.1f}W for {self.exposure_time_ms}ms')
            
            # Maintain beam for exposure duration
            time.sleep(self.exposure_time_ms / 1000.0)
            
            # SIMULATED: Disable laser beam
            self.laser_interface['beam_enabled'] = False
            self.laser_interface['power_output'] = 0.0
            
            fire_duration = (time.time() - start_time) * 1000
            self.get_logger().info(f'Laser pulse completed in {fire_duration:.1f}ms')
            
            self.current_state = LaserState.STANDBY
            return True
            
        except Exception as e:
            self.get_logger().error(f'Laser firing failed: {e}')
            self.emergency_shutdown()
            return False
            
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop signal"""
        if msg.data:
            self.emergency_stop_active = True
            self.current_state = LaserState.EMERGENCY_STOP
            self.emergency_shutdown()
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        else:
            self.emergency_stop_active = False
            self.current_state = LaserState.DISABLED
            self.get_logger().info('Emergency stop released')
            
    def safety_enable_callback(self, msg: Bool):
        """Handle safety enable signal"""
        if msg.data and not self.emergency_stop_active:
            self.laser_enabled = True
            self._safety_confirmed = True
            self.current_state = LaserState.STANDBY
            self.get_logger().info('Laser system enabled with safety confirmation')
        else:
            self.laser_enabled = False
            self.current_state = LaserState.DISABLED
            self.get_logger().info('Laser system disabled')
            
    def emergency_shutdown(self):
        """Immediate emergency shutdown of all laser systems"""
        self.laser_interface['beam_enabled'] = False
        self.laser_interface['power_output'] = 0.0
        self.targeting_queue.clear()
        self.get_logger().error('EMERGENCY SHUTDOWN: All laser systems disabled')
        
    def safety_monitor_loop(self):
        """Continuous safety monitoring loop"""
        while rclpy.ok():
            try:
                # Monitor for safety violations
                if self.laser_interface['beam_enabled']:
                    # Check maximum fire duration
                    fire_duration = (time.time() * 1000) - self.last_fire_time
                    if fire_duration > self.max_fire_duration_ms:
                        self.get_logger().error('Maximum fire duration exceeded')
                        self.emergency_shutdown()
                        
                # Monitor system health
                if self.current_state == LaserState.EMERGENCY_STOP:
                    time.sleep(5.0)  # Extended monitoring during emergency
                else:
                    time.sleep(0.1)  # 10Hz monitoring rate
                    
            except Exception as e:
                self.get_logger().error(f'Safety monitor error: {e}')
                time.sleep(1.0)
                
    def manual_targeting_callback(self, request, response):
        """Handle manual targeting service requests"""
        try:
            if not self.laser_enabled:
                response.success = False
                response.message = "Laser system not enabled"
                return response
                
            target = {
                'position': (request.target_x, request.target_y),
                'timestamp': time.time(),
                'priority': 2.0  # Manual targets get higher priority
            }
            
            if self.execute_weed_targeting(target):
                response.success = True
                response.message = f"Successfully targeted weed at ({request.target_x}, {request.target_y})"
                response.execution_time_ms = self.exposure_time_ms
            else:
                response.success = False
                response.message = "Failed to execute targeting"
                
        except Exception as e:
            response.success = False
            response.message = f"Targeting error: {str(e)}"
            
        return response
        
    def laser_calibration_callback(self, request, response):
        """Handle laser calibration service requests"""
        try:
            self.current_state = LaserState.CALIBRATING
            self.get_logger().info('Starting laser calibration sequence')
            
            # Perform calibration routine
            calibration_successful = self.perform_calibration_routine(request)
            
            if calibration_successful:
                response.success = True
                response.message = "Laser calibration completed successfully"
                response.targeting_accuracy_mm = self.targeting_precision_mm
            else:
                response.success = False
                response.message = "Laser calibration failed"
                
            self.current_state = LaserState.STANDBY
            
        except Exception as e:
            response.success = False
            response.message = f"Calibration error: {str(e)}"
            self.current_state = LaserState.STANDBY
            
        return response
        
    def perform_calibration_routine(self, request) -> bool:
        """Perform laser targeting calibration"""
        # SIMULATED: Calibration routine
        self.get_logger().info('Performing targeting calibration...')
        
        # Test servo positioning accuracy
        test_positions = [(0, 0), (10, 10), (-10, -10), (15, -15)]
        
        for pan, tilt in test_positions:
            if not self.position_targeting_servos(pan, tilt):
                return False
            time.sleep(0.5)
            
        self.get_logger().info('Calibration routine completed')
        return True
        
    def publish_status(self):
        """Publish laser system status"""
        status_msg = String()
        status_msg.data = f"State: {self.current_state.name}, Enabled: {self.laser_enabled}, Weeds Targeted: {self.total_weeds_targeted}"
        self.laser_status_pub.publish(status_msg)
        
        if self.log_targeting_stats:
            stats_msg = String()
            stats_msg.data = f"Queue Length: {len(self.targeting_queue)}, Power: {self.laser_power_pct}%"
            self.targeting_stats_pub.publish(stats_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        laser_incisor = LaserWeedIncisor()
        rclpy.spin(laser_incisor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in laser weed incisor: {e}')
    finally:
        if 'laser_incisor' in locals():
            laser_incisor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()