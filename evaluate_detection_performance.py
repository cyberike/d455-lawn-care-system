#!/usr/bin/env python3
"""
Lightweight Evaluation Loop for D455 Lawn Care Detection Performance
Quantifies grass and obstacle detection quality using recorded bag data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
import json
import time
from datetime import datetime
from collections import defaultdict, deque
import matplotlib.pyplot as plt
import argparse

# ROS 2 message types
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32

class DetectionEvaluator(Node):
    def __init__(self, evaluation_duration=30.0, output_dir=None):
        super().__init__('detection_evaluator')
        
        self.bridge = CvBridge()
        self.evaluation_duration = evaluation_duration
        self.start_time = time.time()
        self.output_dir = output_dir or f"evaluation_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Data storage
        self.metrics = {
            'grass_detection': {
                'coverage_history': [],
                'detection_rate': 0,
                'avg_coverage': 0,
                'coverage_variance': 0,
                'detection_gaps': [],
                'frame_count': 0
            },
            'obstacle_detection': {
                'obstacle_counts': [],
                'detection_rate': 0,
                'avg_obstacles': 0,
                'close_obstacle_ratio': 0,
                'detection_consistency': 0,
                'frame_count': 0
            },
            'system_performance': {
                'total_messages': 0,
                'message_rates': defaultdict(list),
                'processing_delays': [],
                'evaluation_duration': 0
            }
        }
        
        # Recent data for consistency analysis
        self.recent_grass_coverage = deque(maxlen=10)
        self.recent_obstacle_counts = deque(maxlen=10)
        
        # Timing tracking
        self.last_grass_msg_time = None
        self.last_obstacle_msg_time = None
        self.grass_msg_times = deque(maxlen=50)
        self.obstacle_msg_times = deque(maxlen=50)
        
        # QoS profile
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers for evaluation
        self.setup_subscribers()
        
        # Evaluation timer
        self.evaluation_timer = self.create_timer(1.0, self.update_metrics)
        self.completion_timer = self.create_timer(
            self.evaluation_duration, 
            self.complete_evaluation
        )
        
        self.get_logger().info(f'Detection Evaluator initialized')
        self.get_logger().info(f'Evaluation duration: {evaluation_duration}s')
        self.get_logger().info(f'Output directory: {self.output_dir}')
    
    def setup_subscribers(self):
        """Setup subscribers for evaluation data"""
        
        # Grass detection metrics
        self.grass_coverage_sub = self.create_subscription(
            Float32,
            '/simple_grass_detector/grass_coverage_percentage',
            self.grass_coverage_callback,
            10
        )
        
        self.grass_status_sub = self.create_subscription(
            String,
            '/simple_grass_detector/grass_status',
            self.grass_status_callback,
            10
        )
        
        self.grass_debug_sub = self.create_subscription(
            Image,
            '/simple_grass_detector/debug_image',
            self.grass_debug_callback,
            self.qos
        )
        
        # Obstacle detection metrics
        self.obstacle_count_sub = self.create_subscription(
            Int32,
            '/simple_obstacle_detector/obstacle_count',
            self.obstacle_count_callback,
            10
        )
        
        self.obstacle_status_sub = self.create_subscription(
            String,
            '/simple_obstacle_detector/obstacle_status',
            self.obstacle_status_callback,
            10
        )
        
        self.obstacle_debug_sub = self.create_subscription(
            Image,
            '/simple_obstacle_detector/debug_image',
            self.obstacle_debug_callback,
            self.qos
        )
    
    def grass_coverage_callback(self, msg):
        """Track grass coverage data"""
        current_time = time.time()
        coverage = msg.data
        
        # Record coverage data
        self.metrics['grass_detection']['coverage_history'].append({
            'timestamp': current_time,
            'coverage': coverage
        })
        self.metrics['grass_detection']['frame_count'] += 1
        
        # Track message timing
        if self.last_grass_msg_time:
            interval = current_time - self.last_grass_msg_time
            self.grass_msg_times.append(interval)
        self.last_grass_msg_time = current_time
        
        # Update recent data for consistency analysis
        self.recent_grass_coverage.append(coverage)
        
        # Update system metrics
        self.metrics['system_performance']['total_messages'] += 1
        self.metrics['system_performance']['message_rates']['grass_coverage'].append(current_time)
    
    def grass_status_callback(self, msg):
        """Track grass detection status"""
        pass  # Status messages logged for completeness
    
    def grass_debug_callback(self, msg):
        """Analyze grass debug images for quality metrics"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Simple quality metrics from debug image
            # Could analyze color distribution, mask quality, etc.
            
        except Exception as e:
            self.get_logger().debug(f'Error processing grass debug image: {e}')
    
    def obstacle_count_callback(self, msg):
        """Track obstacle detection data"""
        current_time = time.time()
        count = msg.data
        
        # Record obstacle data
        self.metrics['obstacle_detection']['obstacle_counts'].append({
            'timestamp': current_time,
            'count': count
        })
        self.metrics['obstacle_detection']['frame_count'] += 1
        
        # Track message timing
        if self.last_obstacle_msg_time:
            interval = current_time - self.last_obstacle_msg_time
            self.obstacle_msg_times.append(interval)
        self.last_obstacle_msg_time = current_time
        
        # Update recent data
        self.recent_obstacle_counts.append(count)
        
        # Update system metrics
        self.metrics['system_performance']['total_messages'] += 1
        self.metrics['system_performance']['message_rates']['obstacle_count'].append(current_time)
    
    def obstacle_status_callback(self, msg):
        """Parse obstacle status for close detection analysis"""
        status = msg.data
        
        # Extract close obstacle info from status string
        if "close" in status:
            try:
                # Parse "X obstacles detected, Y close" format
                parts = status.split(',')
                if len(parts) > 1:
                    close_part = parts[1].strip()
                    close_count = int(close_part.split()[0])
                    # Could store this for detailed analysis
            except:
                pass
    
    def obstacle_debug_callback(self, msg):
        """Analyze obstacle debug images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Could analyze bounding box quality, detection accuracy
            
        except Exception as e:
            self.get_logger().debug(f'Error processing obstacle debug image: {e}')
    
    def update_metrics(self):
        """Update calculated metrics periodically"""
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # Grass detection metrics
        if self.metrics['grass_detection']['coverage_history']:
            coverages = [d['coverage'] for d in self.metrics['grass_detection']['coverage_history']]
            self.metrics['grass_detection']['avg_coverage'] = np.mean(coverages)
            self.metrics['grass_detection']['coverage_variance'] = np.var(coverages)
            
            if self.grass_msg_times:
                intervals = list(self.grass_msg_times)
                self.metrics['grass_detection']['detection_rate'] = 1.0 / np.mean(intervals) if intervals else 0
        
        # Obstacle detection metrics  
        if self.metrics['obstacle_detection']['obstacle_counts']:
            counts = [d['count'] for d in self.metrics['obstacle_detection']['obstacle_counts']]
            self.metrics['obstacle_detection']['avg_obstacles'] = np.mean(counts)
            
            if self.obstacle_msg_times:
                intervals = list(self.obstacle_msg_times)
                self.metrics['obstacle_detection']['detection_rate'] = 1.0 / np.mean(intervals) if intervals else 0
        
        # Consistency metrics
        if len(self.recent_grass_coverage) > 5:
            recent_std = np.std(list(self.recent_grass_coverage))
            # Lower std deviation = more consistent detection
            self.metrics['grass_detection']['consistency_score'] = max(0, 100 - recent_std * 2)
        
        if len(self.recent_obstacle_counts) > 5:
            recent_std = np.std(list(self.recent_obstacle_counts))
            self.metrics['obstacle_detection']['detection_consistency'] = max(0, 100 - recent_std * 10)
        
        # System performance
        self.metrics['system_performance']['evaluation_duration'] = elapsed
        
        # Progress update
        progress = (elapsed / self.evaluation_duration) * 100
        self.get_logger().info(f'Evaluation progress: {progress:.1f}% ({elapsed:.1f}s/{self.evaluation_duration}s)')
    
    def complete_evaluation(self):
        """Complete evaluation and generate results"""
        self.get_logger().info('Evaluation complete! Generating results...')
        
        # Final metrics calculation
        self.calculate_final_metrics()
        
        # Generate reports
        self.save_raw_data()
        self.generate_performance_report()
        self.create_visualizations()
        
        # Print summary
        self.print_evaluation_summary()
        
        # Shutdown
        self.destroy_node()
        rclpy.shutdown()
    
    def calculate_final_metrics(self):
        """Calculate final performance scores"""
        
        # Grass Detection Score (0-100)
        grass_metrics = self.metrics['grass_detection']
        grass_score = 0
        
        if grass_metrics['frame_count'] > 0:
            # Rate score (target: 8-12 Hz)
            rate = grass_metrics['detection_rate']
            rate_score = max(0, 100 - abs(rate - 10) * 10) if rate > 0 else 0
            
            # Coverage consistency (lower variance = better)
            variance = grass_metrics['coverage_variance']
            consistency_score = max(0, 100 - variance) if variance < 100 else 0
            
            # Detection reliability (frames received vs expected)
            expected_frames = self.evaluation_duration * 10  # 10 Hz target
            reliability_score = min(100, (grass_metrics['frame_count'] / expected_frames) * 100)
            
            grass_score = (rate_score * 0.3 + consistency_score * 0.4 + reliability_score * 0.3)
        
        # Obstacle Detection Score (0-100)
        obstacle_metrics = self.metrics['obstacle_detection']
        obstacle_score = 0
        
        if obstacle_metrics['frame_count'] > 0:
            # Rate score (target: 12-18 Hz)
            rate = obstacle_metrics['detection_rate']
            rate_score = max(0, 100 - abs(rate - 15) * 5) if rate > 0 else 0
            
            # Consistency score
            consistency = obstacle_metrics.get('detection_consistency', 0)
            
            # Detection reliability
            expected_frames = self.evaluation_duration * 15  # 15 Hz target
            reliability_score = min(100, (obstacle_metrics['frame_count'] / expected_frames) * 100)
            
            obstacle_score = (rate_score * 0.3 + consistency * 0.4 + reliability_score * 0.3)
        
        # Overall system score
        overall_score = (grass_score + obstacle_score) / 2
        
        # Store scores
        self.metrics['evaluation_scores'] = {
            'grass_detection_score': grass_score,
            'obstacle_detection_score': obstacle_score,
            'overall_score': overall_score,
            'grade': self.get_letter_grade(overall_score)
        }
    
    def get_letter_grade(self, score):
        """Convert numeric score to letter grade"""
        if score >= 90: return 'A'
        elif score >= 80: return 'B'
        elif score >= 70: return 'C'
        elif score >= 60: return 'D'
        else: return 'F'
    
    def save_raw_data(self):
        """Save raw evaluation data"""
        output_file = os.path.join(self.output_dir, 'evaluation_data.json')
        
        # Convert numpy types for JSON serialization
        def convert_numpy(obj):
            if isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            return obj
        
        # Clean data for JSON
        json_data = json.loads(json.dumps(self.metrics, default=convert_numpy))
        
        with open(output_file, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        self.get_logger().info(f'Raw data saved: {output_file}')
    
    def generate_performance_report(self):
        """Generate human-readable performance report"""
        report_file = os.path.join(self.output_dir, 'performance_report.txt')
        
        with open(report_file, 'w') as f:
            f.write("D455 Lawn Care Detection Performance Report\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Evaluation Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Duration: {self.evaluation_duration}s\n\n")
            
            # Overall scores
            scores = self.metrics.get('evaluation_scores', {})
            f.write("OVERALL PERFORMANCE\n")
            f.write("-" * 20 + "\n")
            f.write(f"Overall Score: {scores.get('overall_score', 0):.1f}/100 (Grade: {scores.get('grade', 'N/A')})\n")
            f.write(f"Grass Detection: {scores.get('grass_detection_score', 0):.1f}/100\n")
            f.write(f"Obstacle Detection: {scores.get('obstacle_detection_score', 0):.1f}/100\n\n")
            
            # Grass detection details
            grass = self.metrics['grass_detection']
            f.write("GRASS DETECTION ANALYSIS\n")
            f.write("-" * 25 + "\n")
            f.write(f"Messages Received: {grass['frame_count']}\n")
            f.write(f"Detection Rate: {grass['detection_rate']:.1f} Hz\n")
            f.write(f"Average Coverage: {grass['avg_coverage']:.1f}%\n")
            f.write(f"Coverage Variance: {grass['coverage_variance']:.2f}\n")
            f.write(f"Consistency Score: {grass.get('consistency_score', 0):.1f}/100\n\n")
            
            # Obstacle detection details
            obstacle = self.metrics['obstacle_detection']
            f.write("OBSTACLE DETECTION ANALYSIS\n")
            f.write("-" * 28 + "\n")
            f.write(f"Messages Received: {obstacle['frame_count']}\n")
            f.write(f"Detection Rate: {obstacle['detection_rate']:.1f} Hz\n")
            f.write(f"Average Obstacles: {obstacle['avg_obstacles']:.1f}\n")
            f.write(f"Detection Consistency: {obstacle['detection_consistency']:.1f}/100\n\n")
            
            # System performance
            system = self.metrics['system_performance']
            f.write("SYSTEM PERFORMANCE\n")
            f.write("-" * 18 + "\n")
            f.write(f"Total Messages: {system['total_messages']}\n")
            f.write(f"Evaluation Duration: {system['evaluation_duration']:.1f}s\n\n")
            
            # Recommendations
            f.write("TUNING RECOMMENDATIONS\n")
            f.write("-" * 22 + "\n")
            self.write_recommendations(f)
        
        self.get_logger().info(f'Performance report saved: {report_file}')
    
    def write_recommendations(self, f):
        """Write tuning recommendations based on results"""
        grass = self.metrics['grass_detection']
        obstacle = self.metrics['obstacle_detection']
        
        # Grass detection recommendations
        if grass['detection_rate'] < 8:
            f.write("• Increase grass detection rate (reduce timer period)\n")
        elif grass['detection_rate'] > 12:
            f.write("• Decrease grass detection rate for better performance\n")
        
        if grass['coverage_variance'] > 50:
            f.write("• High grass coverage variance - consider adjusting HSV thresholds\n")
        
        if grass.get('consistency_score', 0) < 70:
            f.write("• Poor detection consistency - review morphological operations\n")
        
        # Obstacle detection recommendations
        if obstacle['detection_rate'] < 12:
            f.write("• Increase obstacle detection rate\n")
        elif obstacle['detection_rate'] > 18:
            f.write("• Decrease obstacle detection rate for efficiency\n")
        
        if obstacle['detection_consistency'] < 70:
            f.write("• Inconsistent obstacle detection - review depth thresholds\n")
        
        # General recommendations
        if grass['frame_count'] == 0:
            f.write("• No grass detection messages received - check node connectivity\n")
        if obstacle['frame_count'] == 0:
            f.write("• No obstacle detection messages received - check node connectivity\n")
    
    def create_visualizations(self):
        """Create performance visualization plots"""
        try:
            # Grass coverage over time
            if self.metrics['grass_detection']['coverage_history']:
                plt.figure(figsize=(12, 8))
                
                # Grass coverage plot
                plt.subplot(2, 2, 1)
                history = self.metrics['grass_detection']['coverage_history']
                times = [(d['timestamp'] - self.start_time) for d in history]
                coverages = [d['coverage'] for d in history]
                plt.plot(times, coverages, 'g-', linewidth=2)
                plt.title('Grass Coverage Over Time')
                plt.xlabel('Time (s)')
                plt.ylabel('Coverage (%)')
                plt.grid(True)
                
                # Coverage distribution
                plt.subplot(2, 2, 2)
                plt.hist(coverages, bins=20, alpha=0.7, color='green')
                plt.title('Grass Coverage Distribution')
                plt.xlabel('Coverage (%)')
                plt.ylabel('Frequency')
                plt.grid(True)
                
                # Obstacle count over time
                plt.subplot(2, 2, 3)
                if self.metrics['obstacle_detection']['obstacle_counts']:
                    history = self.metrics['obstacle_detection']['obstacle_counts']
                    times = [(d['timestamp'] - self.start_time) for d in history]
                    counts = [d['count'] for d in history]
                    plt.plot(times, counts, 'r-', linewidth=2)
                    plt.title('Obstacle Count Over Time')
                    plt.xlabel('Time (s)')
                    plt.ylabel('Obstacle Count')
                    plt.grid(True)
                
                # Performance scores
                plt.subplot(2, 2, 4)
                scores = self.metrics.get('evaluation_scores', {})
                categories = ['Grass\nDetection', 'Obstacle\nDetection', 'Overall']
                values = [
                    scores.get('grass_detection_score', 0),
                    scores.get('obstacle_detection_score', 0),
                    scores.get('overall_score', 0)
                ]
                colors = ['green', 'red', 'blue']
                bars = plt.bar(categories, values, color=colors, alpha=0.7)
                plt.title('Performance Scores')
                plt.ylabel('Score (0-100)')
                plt.ylim(0, 100)
                plt.grid(True, axis='y')
                
                # Add score labels on bars
                for bar, value in zip(bars, values):
                    plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                            f'{value:.1f}', ha='center', va='bottom')
                
                plt.tight_layout()
                plot_file = os.path.join(self.output_dir, 'performance_plots.png')
                plt.savefig(plot_file, dpi=300, bbox_inches='tight')
                plt.close()
                
                self.get_logger().info(f'Visualization saved: {plot_file}')
                
        except Exception as e:
            self.get_logger().error(f'Error creating visualizations: {e}')
    
    def print_evaluation_summary(self):
        """Print evaluation summary to console"""
        scores = self.metrics.get('evaluation_scores', {})
        
        print("\n" + "="*60)
        print("D455 LAWN CARE DETECTION EVALUATION RESULTS")
        print("="*60)
        print(f"Overall Performance: {scores.get('overall_score', 0):.1f}/100 (Grade: {scores.get('grade', 'N/A')})")
        print(f"Grass Detection Score: {scores.get('grass_detection_score', 0):.1f}/100")
        print(f"Obstacle Detection Score: {scores.get('obstacle_detection_score', 0):.1f}/100")
        print(f"\nResults saved to: {self.output_dir}")
        print("="*60 + "\n")


def main(args=None):
    parser = argparse.ArgumentParser(description='Evaluate D455 lawn care detection performance')
    parser.add_argument('--duration', '-d', type=float, default=30.0,
                       help='Evaluation duration in seconds (default: 30)')
    parser.add_argument('--output', '-o', type=str,
                       help='Output directory for results')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        evaluator = DetectionEvaluator(
            evaluation_duration=args.duration,
            output_dir=args.output
        )
        
        print(f"\nStarting D455 Lawn Care Detection Evaluation")
        print(f"Duration: {args.duration} seconds")
        print(f"Make sure detection nodes and bag playback are running...")
        print("Press Ctrl+C to stop early\n")
        
        rclpy.spin(evaluator)
        
    except KeyboardInterrupt:
        print("\nEvaluation interrupted by user")
    except Exception as e:
        print(f"Evaluation error: {e}")
    finally:
        if 'evaluator' in locals():
            evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()