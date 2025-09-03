#!/usr/bin/env python3
"""
Laser Safety Monitor Node
Continuous safety monitoring for laser weed incisor system
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Optional

# ROS 2 message types
from std_msgs.msg import Bool, String, Float32, Header
from sensor_msgs.msg import Temperature, Range
from geometry_msgs.msg import Point32
from d455_lawn_care.msg import LaserStatus

class SafetyLevel(Enum):
    SAFE = 0
    WARNING = 1
    DANGER = 2
    EMERGENCY = 3

@dataclass
class SafetyZone:
    center_x: float = 0.0
    center_y: float = 0.0
    radius_m: float = 2.0
    height_m: float = 0.5

class LaserSafetyMonitor(Node):
    def __init__(self):
        super().__init__('laser_safety_monitor')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Safety zone parameters
                ('safety_zone_radius_m', 2.0),
                ('operator_distance_min_m', 5.0), 
                ('max_ambient_temperature_c', 50.0),
                ('max_laser_temperature_c', 70.0),
                
                # Monitoring parameters
                ('proximity_sensor_enabled', False),
                ('temperature_monitoring_enabled', True),
                ('safety_zone_monitoring', True),
                ('emergency_stop_enabled', True),
                
                # Response parameters
                ('auto_emergency_stop', True),
                ('safety_violation_timeout_s', 5.0),
                ('monitoring_frequency_hz', 10.0),
            ]
        )
        
        # Get parameters
        self.update_parameters()
        
        # Safety state
        self.current_safety_level = SafetyLevel.SAFE
        self.active_warnings = []
        self.safety_violations = []
        self.emergency_stop_triggered = False
        self.last_safety_check = time.time()
        
        # Safety zone definition
        self.safety_zone = SafetyZone(
            radius_m=self.safety_zone_radius_m
        )
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers for monitoring inputs
        self.laser_status_sub = self.create_subscription(
            LaserStatus,
            '/laser_weed_incisor/laser_status',
            self.laser_status_callback,
            self.reliable_qos
        )
        
        if self.temperature_monitoring_enabled:
            self.temperature_sub = self.create_subscription(
                Temperature,
                '~/system_temperature',
                self.temperature_callback,
                10
            )
            
        if self.proximity_sensor_enabled:
            self.proximity_sub = self.create_subscription(
                Range,
                '~/proximity_sensor',
                self.proximity_callback,
                10
            )
        
        # Create publishers for safety signals
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/laser_weed_incisor/emergency_stop',
            self.reliable_qos
        )
        
        self.safety_status_pub = self.create_publisher(
            String,
            '~/safety_status',
            10
        )
        
        self.safety_warnings_pub = self.create_publisher(
            String,
            '~/safety_warnings',
            10
        )
        
        # Safety monitoring timer
        self.monitoring_timer = self.create_timer(
            1.0 / self.monitoring_frequency_hz,
            self.safety_monitoring_callback
        )
        
        # Safety violation tracking
        self.safety_violation_start_time = None
        self.temperature_readings = []
        self.proximity_readings = []
        
        self.get_logger().info('Laser Safety Monitor initialized')
        self.get_logger().info(f'Safety zone: {self.safety_zone_radius_m}m radius')
        
    def update_parameters(self):
        """Update parameters from ROS parameter server"""
        self.safety_zone_radius_m = self.get_parameter('safety_zone_radius_m').value
        self.operator_distance_min_m = self.get_parameter('operator_distance_min_m').value
        self.max_ambient_temperature_c = self.get_parameter('max_ambient_temperature_c').value
        self.max_laser_temperature_c = self.get_parameter('max_laser_temperature_c').value
        
        self.proximity_sensor_enabled = self.get_parameter('proximity_sensor_enabled').value
        self.temperature_monitoring_enabled = self.get_parameter('temperature_monitoring_enabled').value
        self.safety_zone_monitoring = self.get_parameter('safety_zone_monitoring').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        
        self.auto_emergency_stop = self.get_parameter('auto_emergency_stop').value
        self.safety_violation_timeout_s = self.get_parameter('safety_violation_timeout_s').value
        self.monitoring_frequency_hz = self.get_parameter('monitoring_frequency_hz').value
        
    def laser_status_callback(self, msg: LaserStatus):
        """Monitor laser system status for safety violations"""
        # Check for laser system faults
        if msg.emergency_stop_active:
            self.add_safety_violation("Emergency stop is active")
            
        if not msg.safety_interlocks_ok:
            self.add_safety_violation("Safety interlocks not OK")
            
        # Check temperature limits
        if msg.system_temperature_celsius > self.max_laser_temperature_c:
            self.add_safety_violation(f"Laser temperature too high: {msg.system_temperature_celsius}°C")
            
        # Check for excessive targeting queue (system overload)
        if msg.targeting_queue_length > 50:
            self.add_warning("High targeting queue length - system may be overloaded")
            
        # Check for safety warnings from laser system
        for warning in msg.active_warnings:
            self.add_warning(f"Laser system: {warning}")
            
        for violation in msg.safety_violations:
            self.add_safety_violation(f"Laser system: {violation}")
            
    def temperature_callback(self, msg: Temperature):
        """Monitor ambient temperature"""
        self.temperature_readings.append({
            'temperature': msg.temperature,
            'timestamp': time.time()
        })
        
        # Keep only recent readings (last 30 seconds)
        cutoff_time = time.time() - 30.0
        self.temperature_readings = [
            reading for reading in self.temperature_readings 
            if reading['timestamp'] > cutoff_time
        ]
        
        # Check temperature limits
        if msg.temperature > self.max_ambient_temperature_c:
            self.add_safety_violation(f"Ambient temperature too high: {msg.temperature}°C")
            
    def proximity_callback(self, msg: Range):
        """Monitor proximity sensors for safety zone intrusions"""
        self.proximity_readings.append({
            'range': msg.range,
            'timestamp': time.time()
        })
        
        # Keep only recent readings
        cutoff_time = time.time() - 10.0
        self.proximity_readings = [
            reading for reading in self.proximity_readings
            if reading['timestamp'] > cutoff_time
        ]
        
        # Check for safety zone intrusion
        if msg.range < self.safety_zone_radius_m:
            self.add_safety_violation(f"Safety zone intrusion detected: {msg.range:.2f}m")
            
    def safety_monitoring_callback(self):
        """Main safety monitoring callback"""
        current_time = time.time()
        
        try:
            # Perform comprehensive safety checks
            self.check_system_health()
            self.check_environmental_conditions()
            self.check_safety_zone()
            
            # Evaluate overall safety level
            self.evaluate_safety_level()
            
            # Handle safety violations
            self.handle_safety_violations()
            
            # Publish safety status
            self.publish_safety_status()
            
            self.last_safety_check = current_time
            
        except Exception as e:
            self.get_logger().error(f'Safety monitoring error: {e}')
            self.trigger_emergency_stop("Safety monitor malfunction")
            
    def check_system_health(self):
        """Check overall system health"""
        current_time = time.time()
        
        # Check if we're receiving laser status updates
        if hasattr(self, 'last_laser_status_time'):
            status_age = current_time - self.last_laser_status_time
            if status_age > 5.0:  # 5 second timeout
                self.add_warning("Laser status communication timeout")
                
        # Check monitoring loop frequency
        if hasattr(self, 'last_safety_check'):
            check_interval = current_time - self.last_safety_check
            expected_interval = 1.0 / self.monitoring_frequency_hz
            if check_interval > expected_interval * 2:
                self.add_warning("Safety monitoring frequency degraded")
                
    def check_environmental_conditions(self):
        """Check environmental safety conditions"""
        if not self.temperature_readings:
            return
            
        # Check for temperature trends
        recent_temps = [r['temperature'] for r in self.temperature_readings[-5:]]
        if len(recent_temps) >= 3:
            temp_trend = (recent_temps[-1] - recent_temps[0]) / len(recent_temps)
            if temp_trend > 2.0:  # Rising more than 2°C per reading
                self.add_warning("Rapid temperature increase detected")
                
    def check_safety_zone(self):
        """Check safety zone for intrusions"""
        if not self.safety_zone_monitoring or not self.proximity_readings:
            return
            
        # Check recent proximity readings
        recent_readings = [r for r in self.proximity_readings if time.time() - r['timestamp'] < 1.0]
        
        for reading in recent_readings:
            if reading['range'] < self.operator_distance_min_m:
                self.add_safety_violation(f"Operator too close: {reading['range']:.2f}m")
                
    def add_warning(self, warning: str):
        """Add a safety warning"""
        if warning not in self.active_warnings:
            self.active_warnings.append(warning)
            self.get_logger().warn(f"SAFETY WARNING: {warning}")
            
    def add_safety_violation(self, violation: str):
        """Add a safety violation"""
        if violation not in self.safety_violations:
            self.safety_violations.append(violation)
            self.get_logger().error(f"SAFETY VIOLATION: {violation}")
            
            if self.safety_violation_start_time is None:
                self.safety_violation_start_time = time.time()
                
    def evaluate_safety_level(self):
        """Evaluate overall safety level"""
        if self.safety_violations:
            if any("Emergency" in v or "temperature" in v.lower() for v in self.safety_violations):
                self.current_safety_level = SafetyLevel.EMERGENCY
            else:
                self.current_safety_level = SafetyLevel.DANGER
        elif self.active_warnings:
            self.current_safety_level = SafetyLevel.WARNING
        else:
            self.current_safety_level = SafetyLevel.SAFE
            
    def handle_safety_violations(self):
        """Handle active safety violations"""
        current_time = time.time()
        
        # Clear expired warnings
        self.active_warnings = [w for w in self.active_warnings if self.is_warning_still_active(w)]
        
        # Handle persistent safety violations
        if self.safety_violations:
            if self.safety_violation_start_time is not None:
                violation_duration = current_time - self.safety_violation_start_time
                
                if violation_duration > self.safety_violation_timeout_s:
                    if self.auto_emergency_stop and not self.emergency_stop_triggered:
                        self.trigger_emergency_stop("Safety violation timeout exceeded")
        else:
            # Clear violation tracking if no active violations
            self.safety_violation_start_time = None
            
        # Reset safety violations list (they'll be re-added if still active)
        self.safety_violations = []
        
    def is_warning_still_active(self, warning: str) -> bool:
        """Check if a warning condition is still active"""
        # Implement logic to determine if specific warnings are still relevant
        # For now, clear warnings after each check cycle
        return False
        
    def trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop"""
        if not self.emergency_stop_triggered:
            self.emergency_stop_triggered = True
            
            # Publish emergency stop signal
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)
            
            self.get_logger().error(f"EMERGENCY STOP TRIGGERED: {reason}")
            
    def publish_safety_status(self):
        """Publish current safety status"""
        status_msg = String()
        status_msg.data = f"Safety Level: {self.current_safety_level.name}"
        
        if self.active_warnings:
            status_msg.data += f", Warnings: {len(self.active_warnings)}"
            
        if self.safety_violations:
            status_msg.data += f", Violations: {len(self.safety_violations)}"
            
        self.safety_status_pub.publish(status_msg)
        
        # Publish detailed warnings if any
        if self.active_warnings or self.safety_violations:
            warnings_msg = String()
            all_issues = self.active_warnings + self.safety_violations
            warnings_msg.data = "; ".join(all_issues)
            self.safety_warnings_pub.publish(warnings_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        safety_monitor = LaserSafetyMonitor()
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in laser safety monitor: {e}')
    finally:
        if 'safety_monitor' in locals():
            safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()