#!/usr/bin/env python3
"""
D455 Camera and Processing Pipeline Performance Analyzer
Comprehensive analysis of camera feed rates, processing latency, and system efficiency
"""

import rclpy
from rclpy.node import Node
import time
import psutil
import threading
from collections import deque, defaultdict
import numpy as np
import subprocess
import os

# ROS 2 message types
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Int32, String
from nav_msgs.msg import OccupancyGrid

class CameraPerformanceAnalyzer(Node):
    def __init__(self):
        super().__init__('camera_performance_analyzer')
        
        # Performance tracking
        self.analysis_duration = 60.0  # 60 second analysis window
        self.start_time = time.time()
        
        # Message timing data
        self.message_times = {
            'color_image': deque(maxlen=200),
            'depth_image': deque(maxlen=200),
            'infra1_image': deque(maxlen=200),
            'infra2_image': deque(maxlen=200),
            'grass_coverage': deque(maxlen=200),
            'obstacle_count': deque(maxlen=200),
            'grass_costmap': deque(maxlen=200),
            'obstacle_costmap': deque(maxlen=200)
        }
        
        # Message counters
        self.message_counts = defaultdict(int)
        self.total_bytes = defaultdict(int)
        
        # Latency tracking (processing delays)
        self.processing_latencies = {
            'grass_detection': deque(maxlen=100),
            'obstacle_detection': deque(maxlen=100),
            'costmap_generation': deque(maxlen=100)
        }
        
        # System resource tracking
        self.cpu_samples = deque(maxlen=300)  # 5 minutes worth at 1Hz
        self.memory_samples = deque(maxlen=300)
        self.network_samples = deque(maxlen=300)
        
        # Image size tracking
        self.image_dimensions = {}
        self.compression_ratios = {}
        
        # Setup subscribers
        self.setup_subscribers()
        
        # System monitoring timer
        self.system_monitor_timer = self.create_timer(1.0, self.monitor_system_resources)
        
        # Analysis timer
        self.analysis_timer = self.create_timer(10.0, self.periodic_analysis)
        
        # Final analysis timer
        self.final_timer = self.create_timer(self.analysis_duration, self.generate_final_analysis)
        
        self.get_logger().info('🔍 Camera Performance Analyzer Started')
        self.get_logger().info(f'Analysis duration: {self.analysis_duration}s')
        self.print_monitoring_info()
    
    def print_monitoring_info(self):
        """Print information about what's being monitored"""
        print("\n" + "="*70)
        print("📊 D455 CAMERA & PIPELINE PERFORMANCE ANALYSIS")
        print("="*70)
        print("MONITORING:")
        print("📷 Camera Streams:")
        print("   • Color image feed (1280x720 @ 30fps target)")
        print("   • Depth image feed (aligned)")
        print("   • Infrared streams (stereo pair)")
        print("")
        print("🧠 Processing Pipeline:")
        print("   • Grass detection processing latency") 
        print("   • Obstacle detection processing latency")
        print("   • Costmap generation performance")
        print("")
        print("💻 System Resources:")
        print("   • CPU utilization")
        print("   • Memory usage")
        print("   • Network throughput")
        print("")
        print("📈 Metrics:")
        print("   • Message rates (Hz)")
        print("   • Processing latencies (ms)")
        print("   • Data throughput (MB/s)")
        print("   • Frame drops and timing jitter")
        print("="*70 + "\n")
    
    def setup_subscribers(self):
        """Setup subscribers for performance monitoring"""
        
        # D455 camera streams
        self.color_sub = self.create_subscription(
            Image, '/d455/d455_camera/color/image_raw',
            lambda msg: self.image_callback(msg, 'color_image'),
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image, '/d455/d455_camera/aligned_depth_to_color/image_raw',
            lambda msg: self.image_callback(msg, 'depth_image'),
            10
        )
        
        self.infra1_sub = self.create_subscription(
            Image, '/d455/d455_camera/infra1/image_rect_raw',
            lambda msg: self.image_callback(msg, 'infra1_image'),
            10
        )
        
        self.infra2_sub = self.create_subscription(
            Image, '/d455/d455_camera/infra2/image_rect_raw',
            lambda msg: self.image_callback(msg, 'infra2_image'),
            10
        )
        
        # Camera info for metadata
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/d455/d455_camera/color/camera_info',
            self.camera_info_callback, 10
        )
        
        # Detection outputs
        self.grass_coverage_sub = self.create_subscription(
            Float32, '/simple_grass_detector/grass_coverage_percentage',
            lambda msg: self.detection_callback(msg, 'grass_coverage'),
            10
        )
        
        self.obstacle_count_sub = self.create_subscription(
            Int32, '/simple_obstacle_detector/obstacle_count',
            lambda msg: self.detection_callback(msg, 'obstacle_count'),
            10
        )
        
        # Costmap outputs
        self.grass_costmap_sub = self.create_subscription(
            OccupancyGrid, '/lawn_care/grass_coverage_costmap',
            lambda msg: self.costmap_callback(msg, 'grass_costmap'),
            10
        )
        
        self.obstacle_costmap_sub = self.create_subscription(
            OccupancyGrid, '/lawn_care/obstacle_costmap',
            lambda msg: self.costmap_callback(msg, 'obstacle_costmap'),
            10
        )
    
    def image_callback(self, msg, stream_name):
        """Handle image message for performance tracking"""
        current_time = time.time()
        
        # Record timing
        self.message_times[stream_name].append(current_time)
        self.message_counts[stream_name] += 1
        
        # Calculate message size
        if hasattr(msg, 'data'):
            message_size = len(msg.data)
            self.total_bytes[stream_name] += message_size
        
        # Track image dimensions
        if stream_name not in self.image_dimensions:
            self.image_dimensions[stream_name] = {
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'step': msg.step
            }
        
        # Calculate processing latency (if timestamp available)
        if hasattr(msg.header, 'stamp'):
            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if ros_time > 0:
                latency = current_time - ros_time
                if latency > 0 and latency < 1.0:  # Sanity check
                    if 'grass' in stream_name.lower():
                        self.processing_latencies['grass_detection'].append(latency * 1000)
                    elif 'depth' in stream_name.lower() or 'infra' in stream_name.lower():
                        self.processing_latencies['obstacle_detection'].append(latency * 1000)
    
    def detection_callback(self, msg, detection_type):
        """Handle detection output messages"""
        current_time = time.time()
        
        self.message_times[detection_type].append(current_time)
        self.message_counts[detection_type] += 1
        
        # Small message size (just the data value)
        self.total_bytes[detection_type] += 8  # Approximate
    
    def costmap_callback(self, msg, costmap_type):
        """Handle costmap messages"""
        current_time = time.time()
        
        self.message_times[costmap_type].append(current_time)
        self.message_counts[costmap_type] += 1
        
        # Calculate costmap size
        if hasattr(msg, 'data'):
            costmap_size = len(msg.data)
            self.total_bytes[costmap_type] += costmap_size
        
        # Calculate costmap generation latency
        if hasattr(msg.header, 'stamp'):
            ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if ros_time > 0:
                latency = current_time - ros_time
                if latency > 0 and latency < 2.0:  # Sanity check
                    self.processing_latencies['costmap_generation'].append(latency * 1000)
    
    def camera_info_callback(self, msg):
        """Handle camera info for metadata"""
        # Just track that we got camera info
        self.message_counts['camera_info'] += 1
    
    def monitor_system_resources(self):
        """Monitor system CPU, memory, and network usage"""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=None)
            self.cpu_samples.append(cpu_percent)
            
            # Memory usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_mb = memory.used / (1024 * 1024)
            self.memory_samples.append({
                'percent': memory_percent,
                'used_mb': memory_mb,
                'available_mb': memory.available / (1024 * 1024)
            })
            
            # Network I/O (simplified)
            try:
                net_io = psutil.net_io_counters()
                self.network_samples.append({
                    'bytes_sent': net_io.bytes_sent,
                    'bytes_recv': net_io.bytes_recv,
                    'timestamp': time.time()
                })
            except:
                pass  # Network monitoring might not be available
                
        except Exception as e:
            self.get_logger().debug(f'System monitoring error: {e}')
    
    def calculate_message_rate(self, stream_name, window_seconds=10.0):
        """Calculate message rate for a stream"""
        if stream_name not in self.message_times:
            return 0.0
            
        times = list(self.message_times[stream_name])
        if len(times) < 2:
            return 0.0
        
        # Filter to window
        current_time = time.time()
        recent_times = [t for t in times if current_time - t <= window_seconds]
        
        if len(recent_times) < 2:
            return 0.0
        
        time_span = recent_times[-1] - recent_times[0]
        if time_span <= 0:
            return 0.0
            
        return (len(recent_times) - 1) / time_span
    
    def calculate_jitter(self, stream_name, window_seconds=10.0):
        """Calculate timing jitter for a stream"""
        if stream_name not in self.message_times:
            return 0.0
            
        times = list(self.message_times[stream_name])
        if len(times) < 3:
            return 0.0
        
        # Filter to window and calculate intervals
        current_time = time.time()
        recent_times = [t for t in times if current_time - t <= window_seconds]
        
        if len(recent_times) < 3:
            return 0.0
        
        intervals = [recent_times[i] - recent_times[i-1] for i in range(1, len(recent_times))]
        
        if len(intervals) == 0:
            return 0.0
            
        return np.std(intervals) * 1000  # Convert to milliseconds
    
    def calculate_throughput_mbps(self, stream_name, window_seconds=10.0):
        """Calculate data throughput in MB/s"""
        if stream_name not in self.total_bytes:
            return 0.0
            
        # This is simplified - we'd need per-message size tracking for accurate windowed throughput
        total_mb = self.total_bytes[stream_name] / (1024 * 1024)
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time <= 0:
            return 0.0
            
        return total_mb / elapsed_time
    
    def periodic_analysis(self):
        """Periodic performance analysis output"""
        elapsed = time.time() - self.start_time
        
        self.get_logger().info(f'📊 Performance Update ({elapsed:.0f}s)')
        
        # Camera stream rates
        color_rate = self.calculate_message_rate('color_image')
        depth_rate = self.calculate_message_rate('depth_image')
        
        if color_rate > 0:
            self.get_logger().info(f'📷 Camera: Color {color_rate:.1f}Hz, Depth {depth_rate:.1f}Hz')
        
        # Detection rates
        grass_rate = self.calculate_message_rate('grass_coverage')
        obstacle_rate = self.calculate_message_rate('obstacle_count')
        
        if grass_rate > 0 or obstacle_rate > 0:
            self.get_logger().info(f'🧠 Detection: Grass {grass_rate:.1f}Hz, Obstacles {obstacle_rate:.1f}Hz')
        
        # System resources
        if self.cpu_samples:
            avg_cpu = np.mean(list(self.cpu_samples)[-10:])  # Last 10 samples
            self.get_logger().info(f'💻 System: CPU {avg_cpu:.1f}%')
        
        # Processing latencies
        if self.processing_latencies['grass_detection']:
            grass_latency = np.mean(list(self.processing_latencies['grass_detection'])[-20:])
            self.get_logger().info(f'⏱️  Latency: Grass detection {grass_latency:.1f}ms')
    
    def generate_final_analysis(self):
        """Generate comprehensive final performance analysis"""
        
        self.get_logger().info('🔍 Generating final performance analysis...')
        
        elapsed_time = time.time() - self.start_time
        
        # Calculate comprehensive statistics
        analysis = self.compute_performance_statistics(elapsed_time)
        
        # Print detailed analysis
        self.print_performance_report(analysis)
        
        # Save detailed report
        self.save_performance_report(analysis)
        
        # Stop analysis
        self.analysis_timer.cancel()
        self.system_monitor_timer.cancel()
        
        self.get_logger().info('📋 Performance analysis complete!')
        
        # Shutdown
        rclpy.shutdown()
    
    def compute_performance_statistics(self, elapsed_time):
        """Compute comprehensive performance statistics"""
        
        analysis = {
            'duration': elapsed_time,
            'camera_streams': {},
            'detection_pipeline': {},
            'system_resources': {},
            'data_throughput': {},
            'performance_scores': {}
        }
        
        # Camera stream analysis
        for stream in ['color_image', 'depth_image', 'infra1_image', 'infra2_image']:
            if self.message_counts[stream] > 0:
                analysis['camera_streams'][stream] = {
                    'total_messages': self.message_counts[stream],
                    'average_rate': self.message_counts[stream] / elapsed_time,
                    'current_rate': self.calculate_message_rate(stream),
                    'jitter_ms': self.calculate_jitter(stream),
                    'dimensions': self.image_dimensions.get(stream, {}),
                    'throughput_mbps': self.calculate_throughput_mbps(stream)
                }
        
        # Detection pipeline analysis
        for detection in ['grass_coverage', 'obstacle_count']:
            if self.message_counts[detection] > 0:
                analysis['detection_pipeline'][detection] = {
                    'total_messages': self.message_counts[detection],
                    'average_rate': self.message_counts[detection] / elapsed_time,
                    'current_rate': self.calculate_message_rate(detection)
                }
        
        # Processing latency analysis
        for process_type, latencies in self.processing_latencies.items():
            if latencies:
                analysis['detection_pipeline'][f'{process_type}_latency'] = {
                    'mean_ms': np.mean(latencies),
                    'std_ms': np.std(latencies),
                    'min_ms': np.min(latencies),
                    'max_ms': np.max(latencies),
                    'p95_ms': np.percentile(latencies, 95),
                    'samples': len(latencies)
                }
        
        # System resource analysis
        if self.cpu_samples:
            analysis['system_resources']['cpu'] = {
                'mean_percent': np.mean(self.cpu_samples),
                'max_percent': np.max(self.cpu_samples),
                'std_percent': np.std(self.cpu_samples)
            }
        
        if self.memory_samples:
            memory_percents = [s['percent'] for s in self.memory_samples]
            memory_used_mb = [s['used_mb'] for s in self.memory_samples]
            
            analysis['system_resources']['memory'] = {
                'mean_percent': np.mean(memory_percents),
                'max_percent': np.max(memory_percents),
                'mean_used_mb': np.mean(memory_used_mb),
                'max_used_mb': np.max(memory_used_mb)
            }
        
        # Performance scores (0-100)
        analysis['performance_scores'] = self.calculate_performance_scores(analysis)
        
        return analysis
    
    def calculate_performance_scores(self, analysis):
        """Calculate overall performance scores"""
        scores = {}
        
        # Camera performance score
        camera_score = 100
        if 'color_image' in analysis['camera_streams']:
            color_rate = analysis['camera_streams']['color_image']['current_rate']
            if color_rate < 20:  # Below 20 fps
                camera_score = max(0, camera_score - (20 - color_rate) * 5)
            
            jitter = analysis['camera_streams']['color_image']['jitter_ms']
            if jitter > 50:  # High jitter
                camera_score = max(0, camera_score - (jitter - 50))
        else:
            camera_score = 0
            
        scores['camera_performance'] = camera_score
        
        # Detection pipeline score
        detection_score = 100
        
        if 'grass_coverage' in analysis['detection_pipeline']:
            grass_rate = analysis['detection_pipeline']['grass_coverage']['current_rate']
            if grass_rate < 8:  # Below target rate
                detection_score = max(0, detection_score - (8 - grass_rate) * 10)
        else:
            detection_score -= 50
            
        if 'grass_detection_latency' in analysis['detection_pipeline']:
            latency = analysis['detection_pipeline']['grass_detection_latency']['mean_ms']
            if latency > 100:  # High latency
                detection_score = max(0, detection_score - (latency - 100) / 10)
        
        scores['detection_performance'] = detection_score
        
        # System resource score
        resource_score = 100
        
        if 'cpu' in analysis['system_resources']:
            cpu_usage = analysis['system_resources']['cpu']['mean_percent']
            if cpu_usage > 80:  # High CPU usage
                resource_score = max(0, resource_score - (cpu_usage - 80) * 2)
        
        if 'memory' in analysis['system_resources']:
            memory_usage = analysis['system_resources']['memory']['mean_percent']
            if memory_usage > 80:  # High memory usage
                resource_score = max(0, resource_score - (memory_usage - 80) * 2)
        
        scores['system_resources'] = resource_score
        
        # Overall score
        scores['overall'] = (camera_score + detection_score + resource_score) / 3
        
        return scores
    
    def print_performance_report(self, analysis):
        """Print detailed performance report"""
        
        print("\n" + "="*80)
        print("📊 D455 CAMERA & PIPELINE PERFORMANCE REPORT")
        print("="*80)
        
        duration = analysis['duration']
        print(f"Analysis Duration: {duration:.1f}s")
        
        # Camera streams
        print(f"\n📷 CAMERA STREAMS")
        print("-" * 40)
        
        for stream, data in analysis['camera_streams'].items():
            rate = data['current_rate']
            jitter = data['jitter_ms'] 
            throughput = data['throughput_mbps']
            dims = data['dimensions']
            
            stream_display = stream.replace('_', ' ').title()
            print(f"{stream_display}:")
            print(f"  Rate: {rate:.1f} Hz (target: 30 Hz)")
            print(f"  Jitter: {jitter:.1f}ms")
            print(f"  Throughput: {throughput:.2f} MB/s")
            
            if dims:
                print(f"  Resolution: {dims.get('width', '?')}x{dims.get('height', '?')}")
                print(f"  Encoding: {dims.get('encoding', '?')}")
            print()
        
        # Detection pipeline
        print(f"🧠 DETECTION PIPELINE")
        print("-" * 40)
        
        for process, data in analysis['detection_pipeline'].items():
            if 'latency' in process:
                process_name = process.replace('_latency', '').replace('_', ' ').title()
                mean_lat = data['mean_ms']
                p95_lat = data['p95_ms']
                print(f"{process_name} Latency:")
                print(f"  Mean: {mean_lat:.1f}ms")
                print(f"  95th percentile: {p95_lat:.1f}ms")
                print(f"  Range: {data['min_ms']:.1f} - {data['max_ms']:.1f}ms")
                print()
            else:
                process_name = process.replace('_', ' ').title()
                rate = data['current_rate']
                total = data['total_messages']
                print(f"{process_name}:")
                print(f"  Rate: {rate:.1f} Hz")
                print(f"  Total messages: {total}")
                print()
        
        # System resources
        print(f"💻 SYSTEM RESOURCES")
        print("-" * 40)
        
        if 'cpu' in analysis['system_resources']:
            cpu = analysis['system_resources']['cpu']
            print(f"CPU Usage:")
            print(f"  Average: {cpu['mean_percent']:.1f}%")
            print(f"  Peak: {cpu['max_percent']:.1f}%")
            print()
        
        if 'memory' in analysis['system_resources']:
            memory = analysis['system_resources']['memory']
            print(f"Memory Usage:")
            print(f"  Average: {memory['mean_percent']:.1f}%")
            print(f"  Peak: {memory['max_percent']:.1f}%")
            print(f"  Average Used: {memory['mean_used_mb']:.0f} MB")
            print()
        
        # Performance scores
        scores = analysis['performance_scores']
        print(f"🏆 PERFORMANCE SCORES")
        print("-" * 40)
        print(f"Camera Performance: {scores['camera_performance']:.0f}/100")
        print(f"Detection Pipeline: {scores['detection_performance']:.0f}/100")
        print(f"System Resources: {scores['system_resources']:.0f}/100")
        print(f"Overall Score: {scores['overall']:.0f}/100")
        
        # Performance grade
        overall = scores['overall']
        if overall >= 90:
            grade, emoji = "A", "🟢"
        elif overall >= 80:
            grade, emoji = "B", "🟡"  
        elif overall >= 70:
            grade, emoji = "C", "🟠"
        elif overall >= 60:
            grade, emoji = "D", "🔴"
        else:
            grade, emoji = "F", "⛔"
            
        print(f"\nOverall Grade: {emoji} {grade}")
        
        # Recommendations
        print(f"\n💡 RECOMMENDATIONS")
        print("-" * 40)
        self.print_recommendations(analysis)
        
        print("="*80 + "\n")
    
    def print_recommendations(self, analysis):
        """Print performance optimization recommendations"""
        
        recommendations = []
        
        # Camera performance recommendations
        if 'color_image' in analysis['camera_streams']:
            color_rate = analysis['camera_streams']['color_image']['current_rate']
            if color_rate < 25:
                recommendations.append("• Increase camera frame rate or reduce resolution")
                
            jitter = analysis['camera_streams']['color_image']['jitter_ms']
            if jitter > 50:
                recommendations.append("• High timing jitter detected - check USB connection/bandwidth")
        
        # Detection performance recommendations  
        if 'grass_detection_latency' in analysis['detection_pipeline']:
            latency = analysis['detection_pipeline']['grass_detection_latency']['mean_ms']
            if latency > 100:
                recommendations.append("• Optimize grass detection algorithm - high processing latency")
        
        if 'grass_coverage' in analysis['detection_pipeline']:
            grass_rate = analysis['detection_pipeline']['grass_coverage']['current_rate']
            if grass_rate < 8:
                recommendations.append("• Increase grass detection processing rate")
        
        # System resource recommendations
        if 'cpu' in analysis['system_resources']:
            cpu_usage = analysis['system_resources']['cpu']['mean_percent']
            if cpu_usage > 80:
                recommendations.append("• High CPU usage - consider reducing processing complexity")
                
        if 'memory' in analysis['system_resources']:
            memory_usage = analysis['system_resources']['memory']['mean_percent']  
            if memory_usage > 80:
                recommendations.append("• High memory usage - optimize buffer sizes")
        
        # General recommendations
        if not analysis['camera_streams']:
            recommendations.append("• No camera streams detected - check D455 connection")
            
        if not analysis['detection_pipeline']:
            recommendations.append("• No detection pipeline active - start detection nodes")
        
        if not recommendations:
            recommendations.append("• System performing well - no optimizations needed")
        
        for rec in recommendations:
            print(rec)
    
    def save_performance_report(self, analysis):
        """Save detailed performance report to file"""
        try:
            import json
            
            # Create filename with timestamp
            timestamp = int(time.time())
            filename = f'/tmp/d455_performance_report_{timestamp}.json'
            
            # Convert numpy types for JSON serialization
            def convert_numpy(obj):
                if isinstance(obj, np.integer):
                    return int(obj)
                elif isinstance(obj, np.floating):
                    return float(obj)
                elif isinstance(obj, np.ndarray):
                    return obj.tolist()
                return obj
            
            # Clean analysis data for JSON
            json_data = json.loads(json.dumps(analysis, default=convert_numpy))
            
            with open(filename, 'w') as f:
                json.dump(json_data, f, indent=2)
            
            self.get_logger().info(f'📄 Performance report saved: {filename}')
            
        except Exception as e:
            self.get_logger().warn(f'Could not save performance report: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        analyzer = CameraPerformanceAnalyzer()
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\n🛑 Performance analysis interrupted")
    except Exception as e:
        print(f"❌ Analysis error: {e}")
    finally:
        if 'analyzer' in locals():
            analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()