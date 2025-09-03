#!/usr/bin/env python3
"""
Simplified Laser Weed Incisor Node for Testing
Basic functionality without custom message dependencies
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import time
import threading
from enum import Enum

# Standard ROS 2 message types only
from std_msgs.msg import Header, Bool, Float32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, PoseStamped

class LaserState(Enum):
    DISABLED = 0
    STANDBY = 1 
    TARGETING = 2
    FIRING = 3
    EMERGENCY_STOP = 4

class LaserWeedIncisorSimple(Node):
    def __init__(self):
        super().__init__('laser_weed_incisor_simple')
        
        # System state
        self.current_state = LaserState.DISABLED
        self.laser_enabled = False
        self.emergency_stop_active = False
        self.total_weeds_targeted = 0
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('laser_power_percentage', 80.0),
                ('exposure_time_ms', 200),
                ('auto_fire_enabled', False),
                ('safety_confirmation_required', True),
                ('emergency_stop_enabled', True),
            ]
        )
        
        # Get parameters
        self.laser_power_pct = self.get_parameter('laser_power_percentage').value
        self.exposure_time_ms = self.get_parameter('exposure_time_ms').value
        self.auto_fire_enabled = self.get_parameter('auto_fire_enabled').value
        self.safety_confirmation_required = self.get_parameter('safety_confirmation_required').value
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').value
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
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
        
        # Initialize simulated laser hardware
        self.laser_interface = {
            'power_output': 0.0,
            'targeting_servos': {'pan': 0.0, 'tilt': 0.0},
            'safety_interlocks': True,
            'beam_enabled': False
        }
        
        # Status update timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Laser Weed Incisor Simple initialized')
        self.get_logger().warning('LASER SAFETY: Class 4 laser system - SIMULATED')
        
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
        self.get_logger().error('EMERGENCY SHUTDOWN: All laser systems disabled')
        
    def simulate_weed_targeting(self):
        """Simulate targeting a weed for testing"""
        if not self.laser_enabled or self.emergency_stop_active:
            self.get_logger().warn('Cannot target - laser not enabled or emergency stop active')
            return False
            
        self.get_logger().info('Simulating weed targeting...')
        self.current_state = LaserState.TARGETING
        
        # Simulate servo positioning
        time.sleep(0.1)
        self.laser_interface['targeting_servos']['pan'] = 15.0
        self.laser_interface['targeting_servos']['tilt'] = -10.0
        
        # Simulate laser firing
        self.current_state = LaserState.FIRING
        self.laser_interface['power_output'] = (self.laser_power_pct / 100.0) * 5.0  # 5W max
        self.laser_interface['beam_enabled'] = True
        
        self.get_logger().info(f'SIMULATED LASER FIRING: {self.laser_interface["power_output"]:.1f}W for {self.exposure_time_ms}ms')
        
        # Simulate exposure time
        time.sleep(self.exposure_time_ms / 1000.0)
        
        # Disable laser
        self.laser_interface['beam_enabled'] = False
        self.laser_interface['power_output'] = 0.0
        
        self.current_state = LaserState.STANDBY
        self.total_weeds_targeted += 1
        
        self.get_logger().info('Simulated targeting completed')
        return True
        
    def publish_status(self):
        """Publish laser system status"""
        status_msg = String()
        status_msg.data = (f"State: {self.current_state.name}, "
                          f"Enabled: {self.laser_enabled}, "
                          f"Emergency: {self.emergency_stop_active}, "
                          f"Weeds: {self.total_weeds_targeted}")
        self.laser_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        laser_incisor = LaserWeedIncisorSimple()
        
        # Test sequence after 2 seconds
        def test_sequence():
            time.sleep(2.0)
            laser_incisor.get_logger().info('Starting test sequence...')
            
            # Enable safety
            time.sleep(1.0)
            laser_incisor.safety_enable_callback(Bool(data=True))
            
            # Simulate targeting 3 weeds
            for i in range(3):
                time.sleep(2.0)
                laser_incisor.simulate_weed_targeting()
                
            laser_incisor.get_logger().info('Test sequence completed')
        
        test_thread = threading.Thread(target=test_sequence, daemon=True)
        test_thread.start()
        
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