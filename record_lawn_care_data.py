#!/usr/bin/env python3
"""
Comprehensive Data Recording for D455 Lawn Care System
Records ROS2 bag, saves raw images, and creates video files for Gazebo simulation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import time
import subprocess
from datetime import datetime
import threading
import signal
import sys

# ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo

class LawnCareDataRecorder(Node):
    def __init__(self, output_dir):
        super().__init__('lawn_care_data_recorder')
        
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.recording = True
        
        # Create timestamped directories
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(output_dir, f"lawn_care_session_{timestamp}")
        self.images_dir = os.path.join(self.session_dir, "raw_images")
        self.videos_dir = os.path.join(self.session_dir, "videos")
        
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.videos_dir, exist_ok=True)
        
        # Video writers
        self.video_writers = {}
        self.frame_counts = {}
        
        # Image counters
        self.image_counters = {}
        
        # QoS profile for image topics
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to all important topics
        self.setup_subscribers()
        
        self.get_logger().info(f'Data recorder initialized - saving to: {self.session_dir}')
    
    def setup_subscribers(self):
        """Setup subscribers for all important topics"""
        
        # D455 Camera streams
        self.color_sub = self.create_subscription(
            Image, 
            '/d455/d455_camera/color/image_raw',
            lambda msg: self.image_callback(msg, 'color_raw'),
            self.image_qos
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/aligned_depth_to_color/image_raw',
            lambda msg: self.image_callback(msg, 'depth_aligned'),
            self.image_qos
        )
        
        self.infra1_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/infra1/image_rect_raw',
            lambda msg: self.image_callback(msg, 'infra1'),
            self.image_qos
        )
        
        self.infra2_sub = self.create_subscription(
            Image,
            '/d455/d455_camera/infra2/image_rect_raw',
            lambda msg: self.image_callback(msg, 'infra2'),
            self.image_qos
        )
        
        # Grass detection outputs
        self.grass_debug_sub = self.create_subscription(
            Image,
            '/simple_grass_detector/debug_image',
            lambda msg: self.image_callback(msg, 'grass_debug'),
            self.image_qos
        )
        
        self.grass_mask_sub = self.create_subscription(
            Image,
            '/simple_grass_detector/grass_mask',
            lambda msg: self.image_callback(msg, 'grass_mask'),
            self.image_qos
        )
        
        # Obstacle detection outputs
        self.obstacle_debug_sub = self.create_subscription(
            Image,
            '/simple_obstacle_detector/debug_image',
            lambda msg: self.image_callback(msg, 'obstacle_debug'),
            self.image_qos
        )
    
    def image_callback(self, msg, stream_name):
        """Generic image callback for all streams"""
        if not self.recording:
            return
            
        try:
            # Convert ROS image to OpenCV
            if msg.encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            elif msg.encoding == '16UC1':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                # Convert depth to 8-bit for video recording
                cv_image_8bit = cv2.convertScaleAbs(cv_image, alpha=255.0/65535.0)
                cv_image = cv_image_8bit
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Initialize counter for this stream
            if stream_name not in self.image_counters:
                self.image_counters[stream_name] = 0
            
            # Save raw images (every 10th frame to save disk space)
            if self.image_counters[stream_name] % 10 == 0:
                self.save_raw_image(cv_image, stream_name, self.image_counters[stream_name])
            
            # Record to video
            self.record_to_video(cv_image, stream_name)
            
            self.image_counters[stream_name] += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing {stream_name}: {e}')
    
    def save_raw_image(self, cv_image, stream_name, frame_count):
        """Save individual images"""
        try:
            stream_dir = os.path.join(self.images_dir, stream_name)
            os.makedirs(stream_dir, exist_ok=True)
            
            filename = f"{stream_name}_{frame_count:06d}.png"
            filepath = os.path.join(stream_dir, filename)
            
            cv2.imwrite(filepath, cv_image)
            
        except Exception as e:
            self.get_logger().debug(f'Error saving image {stream_name}: {e}')
    
    def record_to_video(self, cv_image, stream_name):
        """Record images to video files"""
        try:
            # Initialize video writer for this stream if not exists
            if stream_name not in self.video_writers:
                self.init_video_writer(cv_image, stream_name)
            
            # Write frame to video
            if stream_name in self.video_writers and self.video_writers[stream_name] is not None:
                # Ensure image is the right format for video
                if len(cv_image.shape) == 2:  # Grayscale
                    cv_image_color = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                else:
                    cv_image_color = cv_image
                
                self.video_writers[stream_name].write(cv_image_color)
                
                if stream_name not in self.frame_counts:
                    self.frame_counts[stream_name] = 0
                self.frame_counts[stream_name] += 1
            
        except Exception as e:
            self.get_logger().debug(f'Error writing video frame {stream_name}: {e}')
    
    def init_video_writer(self, sample_image, stream_name):
        """Initialize video writer for a stream"""
        try:
            height, width = sample_image.shape[:2]
            
            # Define codec and create VideoWriter
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 15  # Reduced FPS for file size management
            
            video_filename = f"{stream_name}.mp4"
            video_path = os.path.join(self.videos_dir, video_filename)
            
            self.video_writers[stream_name] = cv2.VideoWriter(
                video_path, fourcc, fps, (width, height)
            )
            
            self.get_logger().info(f'Started video recording for {stream_name}: {video_path}')
            
        except Exception as e:
            self.get_logger().error(f'Error initializing video writer for {stream_name}: {e}')
            self.video_writers[stream_name] = None
    
    def stop_recording(self):
        """Stop recording and cleanup"""
        self.recording = False
        
        # Release video writers
        for stream_name, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
                frames = self.frame_counts.get(stream_name, 0)
                self.get_logger().info(f'Finished video {stream_name}: {frames} frames')
        
        self.get_logger().info('Data recording stopped')
    
    def get_recording_stats(self):
        """Get current recording statistics"""
        stats = {}
        for stream_name in self.image_counters:
            stats[stream_name] = {
                'frames_recorded': self.image_counters[stream_name],
                'images_saved': self.image_counters[stream_name] // 10,
                'video_frames': self.frame_counts.get(stream_name, 0)
            }
        return stats


def start_ros2_bag_recording(output_dir):
    """Start ROS2 bag recording in background"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.join(output_dir, f"lawn_care_session_{timestamp}", "rosbag")
    os.makedirs(bag_dir, exist_ok=True)
    
    # Topics to record
    topics_to_record = [
        # D455 Camera data
        '/d455/d455_camera/color/image_raw',
        '/d455/d455_camera/aligned_depth_to_color/image_raw',
        '/d455/d455_camera/color/camera_info',
        '/d455/d455_camera/depth/camera_info',
        '/d455/d455_camera/infra1/image_rect_raw',
        '/d455/d455_camera/infra2/image_rect_raw',
        
        # Grass detection
        '/simple_grass_detector/grass_coverage_percentage',
        '/simple_grass_detector/grass_status',
        '/simple_grass_detector/debug_image',
        '/simple_grass_detector/grass_mask',
        
        # Obstacle detection
        '/simple_obstacle_detector/obstacle_count',
        '/simple_obstacle_detector/obstacle_status',
        '/simple_obstacle_detector/obstacle_markers',
        '/simple_obstacle_detector/debug_image',
        
        # Transforms
        '/tf',
        '/tf_static',
        
        # System
        '/parameter_events',
        '/rosout',
    ]
    
    # Build ros2 bag command
    cmd = [
        'ros2', 'bag', 'record',
        '--output', bag_dir,
        '--storage', 'mcap',  # Use MCAP format for better performance
        '--compression-mode', 'file',
        '--compression-format', 'zstd',
    ] + topics_to_record
    
    print(f"Starting ROS2 bag recording: {bag_dir}")
    print(f"Recording topics: {len(topics_to_record)} topics")
    
    # Start bag recording in background
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return process


def signal_handler(sig, frame, recorder=None, bag_process=None):
    """Handle Ctrl+C gracefully"""
    print('\nStopping data recording...')
    
    if recorder:
        recorder.stop_recording()
        recorder.destroy_node()
    
    if bag_process:
        bag_process.terminate()
        bag_process.wait()
    
    rclpy.shutdown()
    sys.exit(0)


def main():
    print("=== D455 Lawn Care Data Recording System ===")
    
    # Setup output directory
    home_dir = os.path.expanduser("~")
    output_dir = os.path.join(home_dir, "lawn_care_recordings")
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize ROS
    rclpy.init()
    
    # Start ROS2 bag recording
    bag_process = start_ros2_bag_recording(output_dir)
    
    # Create data recorder node
    recorder = LawnCareDataRecorder(output_dir)
    
    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, lambda s, f: signal_handler(s, f, recorder, bag_process))
    
    print(f"Recording data to: {output_dir}")
    print("Recording:")
    print("  - ROS2 bag (all topics)")
    print("  - Raw images (every 10th frame)")
    print("  - Video files (15fps)")
    print("Press Ctrl+C to stop recording")
    
    # Status reporting timer
    def print_stats():
        stats = recorder.get_recording_stats()
        print(f"\n=== Recording Stats ===")
        for stream, data in stats.items():
            print(f"{stream}: {data['frames_recorded']} frames, {data['images_saved']} images, {data['video_frames']} video frames")
        
        # Schedule next stats report
        threading.Timer(30.0, print_stats).start()
    
    # Start stats reporting
    threading.Timer(30.0, print_stats).start()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        signal_handler(None, None, recorder, bag_process)


if __name__ == '__main__':
    main()