#!/usr/bin/env python3
"""
Video Stream Recorder for D455 Lawn Care
Records video from image topics using OpenCV for Gazebo simulation data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import argparse
from datetime import datetime
import threading
import time

from sensor_msgs.msg import Image

class VideoStreamRecorder(Node):
    def __init__(self, topics, output_dir, duration=None):
        super().__init__('video_stream_recorder')
        
        self.bridge = CvBridge()
        self.topics = topics
        self.output_dir = output_dir
        self.duration = duration
        self.start_time = time.time()
        self.recording = True
        
        # Video writers dictionary
        self.video_writers = {}
        self.frame_counts = {}
        
        # Create timestamped output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(output_dir, f"video_recording_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)
        
        # QoS profile for image topics
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setup subscribers for specified topics
        self.subscribers = {}
        for topic in self.topics:
            topic_name = topic.replace('/', '_').strip('_')
            self.subscribers[topic] = self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic_name: self.image_callback(msg, t),
                self.image_qos
            )
        
        self.get_logger().info(f'Video recorder initialized')
        self.get_logger().info(f'Recording {len(self.topics)} topics to: {self.session_dir}')
        if duration:
            self.get_logger().info(f'Recording duration: {duration} seconds')
        
        # Setup duration timer if specified
        if duration:
            self.timer = self.create_timer(duration, self.stop_recording)
    
    def image_callback(self, msg, stream_name):
        """Image callback for video recording"""
        if not self.recording:
            return
            
        try:
            # Convert ROS image to OpenCV
            if msg.encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                # Convert grayscale to BGR for video
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif msg.encoding == '16UC1':
                # Depth image - normalize to 8-bit
                cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                cv_image_norm = cv2.convertScaleAbs(cv_image, alpha=255.0/65535.0)
                cv_image = cv2.applyColorMap(cv_image_norm, cv2.COLORMAP_JET)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Initialize video writer if not exists
            if stream_name not in self.video_writers:
                self.init_video_writer(cv_image, stream_name)
            
            # Write frame to video
            if stream_name in self.video_writers and self.video_writers[stream_name] is not None:
                self.video_writers[stream_name].write(cv_image)
                
                if stream_name not in self.frame_counts:
                    self.frame_counts[stream_name] = 0
                self.frame_counts[stream_name] += 1
            
        except Exception as e:
            self.get_logger().debug(f'Error recording {stream_name}: {e}')
    
    def init_video_writer(self, sample_image, stream_name):
        """Initialize video writer for a stream"""
        try:
            height, width = sample_image.shape[:2]
            
            # Video settings
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 15.0  # 15 fps for good quality/size balance
            
            video_filename = f"{stream_name}.mp4"
            video_path = os.path.join(self.session_dir, video_filename)
            
            self.video_writers[stream_name] = cv2.VideoWriter(
                video_path, fourcc, fps, (width, height)
            )
            
            self.get_logger().info(f'Started recording: {video_filename} ({width}x{height} @ {fps}fps)')
            
        except Exception as e:
            self.get_logger().error(f'Error initializing video writer for {stream_name}: {e}')
            self.video_writers[stream_name] = None
    
    def stop_recording(self):
        """Stop recording and cleanup"""
        self.recording = False
        
        # Release video writers
        total_frames = 0
        for stream_name, writer in self.video_writers.items():
            if writer is not None:
                writer.release()
                frames = self.frame_counts.get(stream_name, 0)
                total_frames += frames
                duration = frames / 15.0  # Assuming 15fps
                self.get_logger().info(f'Completed: {stream_name}.mp4 - {frames} frames ({duration:.1f}s)')
        
        self.get_logger().info(f'Video recording complete - Total frames: {total_frames}')
        self.get_logger().info(f'Videos saved to: {self.session_dir}')
        
        # Create summary file
        self.create_summary()
    
    def create_summary(self):
        """Create recording summary file"""
        summary_path = os.path.join(self.session_dir, 'recording_summary.txt')
        
        with open(summary_path, 'w') as f:
            f.write("D455 Lawn Care Video Recording Summary\n")
            f.write("=" * 40 + "\n\n")
            f.write(f"Recording Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Output Directory: {self.session_dir}\n")
            f.write(f"Topics Recorded: {len(self.topics)}\n\n")
            
            f.write("Video Files:\n")
            f.write("-" * 20 + "\n")
            
            for stream_name, writer in self.video_writers.items():
                if stream_name in self.frame_counts:
                    frames = self.frame_counts[stream_name]
                    duration = frames / 15.0
                    f.write(f"{stream_name}.mp4: {frames} frames ({duration:.1f}s)\n")
            
            f.write(f"\nTopics:\n")
            f.write("-" * 20 + "\n")
            for topic in self.topics:
                f.write(f"  {topic}\n")
            
            f.write("\nUsage for Gazebo Simulation:\n")
            f.write("-" * 30 + "\n")
            f.write("1. Use these videos as reference for lawn environments\n")
            f.write("2. Extract frames for texture mapping\n")
            f.write("3. Use grass detection patterns for simulation validation\n")
            f.write("4. Obstacle detection data for world generation\n")
        
        self.get_logger().info(f'Summary saved: {summary_path}')


def main():
    parser = argparse.ArgumentParser(description='Record video streams from ROS topics')
    parser.add_argument('--duration', '-d', type=int, default=60, 
                       help='Recording duration in seconds (default: 60)')
    parser.add_argument('--output', '-o', type=str, default='~/lawn_care_videos',
                       help='Output directory (default: ~/lawn_care_videos)')
    parser.add_argument('--topics', '-t', nargs='+', 
                       default=[
                           '/d455/d455_camera/color/image_raw',
                           '/simple_grass_detector/debug_image',
                           '/simple_grass_detector/grass_mask',
                           '/simple_obstacle_detector/debug_image'
                       ],
                       help='Topics to record (space separated)')
    
    args = parser.parse_args()
    
    # Expand home directory
    output_dir = os.path.expanduser(args.output)
    os.makedirs(output_dir, exist_ok=True)
    
    print("=== D455 Lawn Care Video Recorder ===")
    print(f"Duration: {args.duration} seconds")
    print(f"Output: {output_dir}")
    print(f"Topics ({len(args.topics)}):")
    for topic in args.topics:
        print(f"  - {topic}")
    print("\nStarting recording...")
    
    # Initialize ROS
    rclpy.init()
    
    try:
        # Create recorder
        recorder = VideoStreamRecorder(args.topics, output_dir, args.duration)
        
        # Status reporting
        def print_status():
            if recorder.recording:
                elapsed = time.time() - recorder.start_time
                print(f"Recording... {elapsed:.0f}s elapsed")
                
                # Print frame counts
                for stream, count in recorder.frame_counts.items():
                    print(f"  {stream}: {count} frames")
                
                if recorder.duration:
                    remaining = recorder.duration - elapsed
                    print(f"  Time remaining: {remaining:.0f}s")
                
                # Schedule next status
                if recorder.recording:
                    threading.Timer(10.0, print_status).start()
        
        # Start status reporting
        threading.Timer(10.0, print_status).start()
        
        # Spin until duration ends or Ctrl+C
        rclpy.spin(recorder)
        
    except KeyboardInterrupt:
        print("\nStopping recording...")
        if 'recorder' in locals():
            recorder.stop_recording()
    finally:
        if 'recorder' in locals():
            recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()