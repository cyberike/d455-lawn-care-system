#!/usr/bin/env python3
"""
Gazebo Parity Analysis Without Simulation
Analyzes real hardware performance and compares against expected simulation metrics
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os

class GazeboParityAnalyzer:
    def __init__(self):
        self.real_data = None
        self.expected_sim_metrics = {
            # Expected simulation performance based on typical Gazebo behavior
            'camera_rates': {
                'color': {'mean': 30.0, 'std': 0.1},  # Perfect frame rate
                'depth': {'mean': 30.0, 'std': 0.1}   # Perfect frame rate
            },
            'detection_rates': {
                'grass': {'mean': 30.0, 'std': 0.1},    # Process every frame
                'obstacle': {'mean': 30.0, 'std': 0.1}  # Process every frame
            },
            'detection_accuracy': {
                'grass': {'mean': 95.0, 'std': 1.0},     # Perfect lighting conditions
                'obstacle': {'mean': 98.0, 'std': 0.5}   # No sensor noise
            },
            'processing_latencies': {
                'color': {'mean': 5.0, 'std': 0.5},      # ms - deterministic processing
                'depth': {'mean': 5.0, 'std': 0.5},      # ms - deterministic processing
                'grass': {'mean': 10.0, 'std': 1.0},     # ms - consistent compute
                'obstacle': {'mean': 15.0, 'std': 1.0}   # ms - consistent compute
            },
            'stability': {
                'color_cv': 0.5,    # Very stable
                'depth_cv': 0.5,    # Very stable
                'grass_cv': 1.0,    # Very stable
                'obstacle_cv': 1.0  # Very stable
            }
        }
    
    def load_real_data(self, json_file):
        """Load real hardware test data"""
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            self.real_data = data.get('real', {})
            return True
        except Exception as e:
            print(f"Error loading real data: {e}")
            return False
    
    def analyze_real_performance(self):
        """Analyze real hardware performance metrics"""
        if not self.real_data:
            return None
        
        analysis = {}
        
        # Camera rates analysis
        if 'camera_rates' in self.real_data:
            analysis['camera_rates'] = {}
            for stream, rates in self.real_data['camera_rates'].items():
                if rates:
                    analysis['camera_rates'][stream] = {
                        'mean': np.mean(rates),
                        'std': np.std(rates),
                        'min': np.min(rates),
                        'max': np.max(rates),
                        'cv': (np.std(rates) / np.mean(rates)) * 100 if np.mean(rates) > 0 else 0
                    }
        
        # Detection rates analysis
        if 'detection_rates' in self.real_data:
            analysis['detection_rates'] = {}
            for detector, rates in self.real_data['detection_rates'].items():
                if rates:
                    analysis['detection_rates'][detector] = {
                        'mean': np.mean(rates),
                        'std': np.std(rates),
                        'min': np.min(rates),
                        'max': np.max(rates),
                        'cv': (np.std(rates) / np.mean(rates)) * 100 if np.mean(rates) > 0 else 0
                    }
        
        # Detection accuracy analysis
        if 'detection_accuracy' in self.real_data:
            analysis['detection_accuracy'] = {}
            for detector, accuracy in self.real_data['detection_accuracy'].items():
                if accuracy:
                    analysis['detection_accuracy'][detector] = {
                        'mean': np.mean(accuracy),
                        'std': np.std(accuracy),
                        'min': np.min(accuracy),
                        'max': np.max(accuracy)
                    }
        
        # Processing latencies analysis
        if 'processing_latencies' in self.real_data:
            analysis['processing_latencies'] = {}
            for stream, latencies in self.real_data['processing_latencies'].items():
                if latencies:
                    # Convert to milliseconds
                    latencies_ms = [l * 1000 for l in latencies]
                    analysis['processing_latencies'][stream] = {
                        'mean': np.mean(latencies_ms),
                        'std': np.std(latencies_ms),
                        'min': np.min(latencies_ms),
                        'max': np.max(latencies_ms)
                    }
        
        return analysis
    
    def compare_metrics(self, real_analysis):
        """Compare real hardware metrics against expected simulation"""
        if not real_analysis:
            return None
        
        comparison = {
            'camera_performance': {},
            'detection_performance': {},
            'latency_performance': {},
            'stability_performance': {},
            'overall_scores': {}
        }
        
        # Camera performance comparison
        for stream in ['color', 'depth']:
            if stream in real_analysis.get('camera_rates', {}):
                real_rate = real_analysis['camera_rates'][stream]['mean']
                expected_rate = self.expected_sim_metrics['camera_rates'][stream]['mean']
                
                comparison['camera_performance'][stream] = {
                    'real': real_rate,
                    'expected_sim': expected_rate,
                    'difference': real_rate - expected_rate,
                    'percentage_diff': ((real_rate - expected_rate) / expected_rate) * 100,
                    'score': min(100, (real_rate / expected_rate) * 100)
                }
        
        # Detection performance comparison
        for detector in ['grass', 'obstacle']:
            # Detection rates
            if detector in real_analysis.get('detection_rates', {}):
                real_rate = real_analysis['detection_rates'][detector]['mean']
                expected_rate = self.expected_sim_metrics['detection_rates'][detector]['mean']
                
                comparison['detection_performance'][f'{detector}_rate'] = {
                    'real': real_rate,
                    'expected_sim': expected_rate,
                    'difference': real_rate - expected_rate,
                    'percentage_diff': ((real_rate - expected_rate) / expected_rate) * 100,
                    'score': min(100, (real_rate / expected_rate) * 100)
                }
            
            # Detection accuracy
            if detector in real_analysis.get('detection_accuracy', {}):
                real_acc = real_analysis['detection_accuracy'][detector]['mean']
                expected_acc = self.expected_sim_metrics['detection_accuracy'][detector]['mean']
                
                comparison['detection_performance'][f'{detector}_accuracy'] = {
                    'real': real_acc,
                    'expected_sim': expected_acc,
                    'difference': real_acc - expected_acc,
                    'percentage_diff': ((real_acc - expected_acc) / expected_acc) * 100,
                    'score': min(100, (real_acc / expected_acc) * 100)
                }
        
        # Latency performance comparison
        for stream in ['color', 'depth', 'grass', 'obstacle']:
            if stream in real_analysis.get('processing_latencies', {}):
                real_latency = real_analysis['processing_latencies'][stream]['mean']
                expected_latency = self.expected_sim_metrics['processing_latencies'][stream]['mean']
                
                # Lower latency is better, so invert the scoring
                comparison['latency_performance'][stream] = {
                    'real': real_latency,
                    'expected_sim': expected_latency,
                    'difference': real_latency - expected_latency,
                    'percentage_diff': ((real_latency - expected_latency) / expected_latency) * 100,
                    'score': min(100, max(0, (expected_latency / real_latency) * 100))
                }
        
        # Stability performance comparison
        for stream in ['color', 'depth']:
            if stream in real_analysis.get('camera_rates', {}):
                real_cv = real_analysis['camera_rates'][stream]['cv']
                expected_cv = self.expected_sim_metrics['stability'][f'{stream}_cv']
                
                # Lower CV is better (more stable)
                comparison['stability_performance'][stream] = {
                    'real_cv': real_cv,
                    'expected_sim_cv': expected_cv,
                    'stability_score': min(100, max(0, (expected_cv / max(real_cv, 0.1)) * 100))
                }
        
        # Calculate overall scores
        all_scores = []
        
        # Collect camera scores
        for stream_data in comparison['camera_performance'].values():
            all_scores.append(stream_data['score'])
        
        # Collect detection scores
        for det_data in comparison['detection_performance'].values():
            all_scores.append(det_data['score'])
        
        # Collect latency scores
        for lat_data in comparison['latency_performance'].values():
            all_scores.append(lat_data['score'])
        
        # Collect stability scores
        for stab_data in comparison['stability_performance'].values():
            all_scores.append(stab_data['stability_score'])
        
        if all_scores:
            comparison['overall_scores'] = {
                'mean_score': np.mean(all_scores),
                'min_score': np.min(all_scores),
                'max_score': np.max(all_scores),
                'grade': self.get_grade(np.mean(all_scores))
            }
        
        return comparison
    
    def get_grade(self, score):
        """Convert numerical score to letter grade"""
        if score >= 90:
            return 'A'
        elif score >= 80:
            return 'B'
        elif score >= 70:
            return 'C'
        elif score >= 60:
            return 'D'
        else:
            return 'F'
    
    def generate_report(self, comparison, real_analysis):
        """Generate comprehensive parity report"""
        
        print("\n" + "="*80)
        print("🎯 GAZEBO PARITY ANALYSIS REPORT")
        print("="*80)
        print(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Analysis: Real Hardware vs Expected Simulation Performance")
        
        if comparison and comparison.get('overall_scores'):
            overall_score = comparison['overall_scores']['mean_score']
            grade = comparison['overall_scores']['grade']
            print(f"\n🏆 OVERALL PARITY SCORE: {overall_score:.1f}/100 (Grade {grade})")
        
        print(f"\n📊 PERFORMANCE COMPARISON:")
        print("-" * 50)
        
        # Camera Performance
        if 'camera_performance' in comparison:
            print(f"\n📷 CAMERA PERFORMANCE:")
            for stream, data in comparison['camera_performance'].items():
                print(f"  {stream.capitalize()} Stream:")
                print(f"    Real: {data['real']:.1f} Hz")
                print(f"    Expected Sim: {data['expected_sim']:.1f} Hz") 
                print(f"    Score: {data['score']:.1f}/100 ({data['percentage_diff']:+.1f}%)")
        
        # Detection Performance
        if 'detection_performance' in comparison:
            print(f"\n🧠 DETECTION PERFORMANCE:")
            for metric, data in comparison['detection_performance'].items():
                print(f"  {metric.replace('_', ' ').title()}:")
                if 'accuracy' in metric:
                    print(f"    Real: {data['real']:.1f}%")
                    print(f"    Expected Sim: {data['expected_sim']:.1f}%")
                else:
                    print(f"    Real: {data['real']:.1f} Hz")
                    print(f"    Expected Sim: {data['expected_sim']:.1f} Hz")
                print(f"    Score: {data['score']:.1f}/100 ({data['percentage_diff']:+.1f}%)")
        
        # Latency Performance
        if 'latency_performance' in comparison:
            print(f"\n⚡ PROCESSING LATENCY:")
            for stream, data in comparison['latency_performance'].items():
                print(f"  {stream.capitalize()}:")
                print(f"    Real: {data['real']:.1f} ms")
                print(f"    Expected Sim: {data['expected_sim']:.1f} ms")
                print(f"    Score: {data['score']:.1f}/100 ({data['percentage_diff']:+.1f}%)")
        
        # Stability Performance  
        if 'stability_performance' in comparison:
            print(f"\n📈 STABILITY ANALYSIS:")
            for stream, data in comparison['stability_performance'].items():
                print(f"  {stream.capitalize()} Stream:")
                print(f"    Real CV: {data['real_cv']:.1f}%")
                print(f"    Expected Sim CV: {data['expected_sim_cv']:.1f}%")
                print(f"    Stability Score: {data['stability_score']:.1f}/100")
        
        # Key Findings
        print(f"\n🔍 KEY FINDINGS:")
        print("-" * 30)
        
        findings = []
        
        # Check camera performance
        if 'camera_performance' in comparison:
            for stream, data in comparison['camera_performance'].items():
                if data['score'] < 50:
                    findings.append(f"❌ {stream.capitalize()} camera severely underperforming ({data['real']:.1f}/{data['expected_sim']:.1f} Hz)")
                elif data['score'] < 80:
                    findings.append(f"⚠️  {stream.capitalize()} camera below expectations ({data['real']:.1f}/{data['expected_sim']:.1f} Hz)")
                else:
                    findings.append(f"✅ {stream.capitalize()} camera performing well ({data['real']:.1f} Hz)")
        
        # Check detection accuracy
        if 'detection_performance' in comparison:
            for metric, data in comparison['detection_performance'].items():
                if 'accuracy' in metric:
                    detector = metric.replace('_accuracy', '')
                    if data['real'] < 1.0:  # Very low accuracy
                        findings.append(f"❌ {detector.capitalize()} detection accuracy critically low ({data['real']:.1f}%)")
                    elif data['real'] < 10.0:
                        findings.append(f"⚠️  {detector.capitalize()} detection needs parameter tuning ({data['real']:.1f}%)")
        
        # Print findings
        for finding in findings:
            print(f"  {finding}")
        
        # Recommendations
        print(f"\n💡 RECOMMENDATIONS:")
        print("-" * 30)
        
        recommendations = []
        
        # Camera rate recommendations
        if 'camera_performance' in comparison:
            for stream, data in comparison['camera_performance'].items():
                if data['real'] < 15:
                    recommendations.append(f"🔧 Increase {stream} camera frame rate (check USB bandwidth/power)")
                elif data['real'] < 25:
                    recommendations.append(f"⚙️  Optimize {stream} camera settings for higher throughput")
        
        # Detection accuracy recommendations
        grass_acc = None
        if 'detection_performance' in comparison:
            for metric, data in comparison['detection_performance'].items():
                if metric == 'grass_accuracy':
                    grass_acc = data['real']
        
        if grass_acc is not None and grass_acc < 5.0:
            recommendations.append("🌱 Retune grass detection HSV parameters for real environment")
            recommendations.append("🌱 Test grass detection with actual outdoor grass scenes")
            recommendations.append("🌱 Consider adaptive color thresholding for varying lighting")
        
        # General recommendations
        recommendations.append("🎮 Implement simulation with Gazebo/Ignition for true parity testing")
        recommendations.append("📊 Run extended tests (5+ minutes) for better statistical analysis")
        
        for rec in recommendations:
            print(f"  {rec}")
        
        # Simulation Readiness Assessment
        print(f"\n🎮 SIMULATION READINESS:")
        print("-" * 30)
        
        if comparison and comparison.get('overall_scores'):
            score = comparison['overall_scores']['mean_score']
            if score >= 80:
                print("✅ System ready for simulation parity testing")
                print("✅ Performance characteristics well understood")
                print("✅ Can proceed with Gazebo integration")
            elif score >= 60:
                print("⚠️  System partially ready for simulation")
                print("⚠️  Address key performance issues first")
                print("⚠️  Consider simulation for parameter optimization")
            else:
                print("❌ System not ready for simulation parity")
                print("❌ Fix fundamental performance issues")
                print("❌ Focus on real hardware optimization first")
        
        print("="*80 + "\n")
        
        return True
    
    def save_analysis(self, comparison, real_analysis, filename=None):
        """Save analysis results to JSON file"""
        if not filename:
            timestamp = int(datetime.now().timestamp())
            filename = f'/tmp/parity_analysis_{timestamp}.json'
        
        analysis_data = {
            'timestamp': datetime.now().isoformat(),
            'real_hardware_analysis': real_analysis,
            'expected_simulation_metrics': self.expected_sim_metrics,
            'parity_comparison': comparison,
            'metadata': {
                'analyzer_version': '1.0',
                'analysis_type': 'gazebo_parity_without_sim'
            }
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(analysis_data, f, indent=2)
            print(f"📁 Analysis saved to: {filename}")
            return filename
        except Exception as e:
            print(f"❌ Failed to save analysis: {e}")
            return None


def main():
    """Main analysis function"""
    analyzer = GazeboParityAnalyzer()
    
    # Look for the most recent parity test data
    test_files = []
    for file in os.listdir('/tmp'):
        if file.startswith('parity_test_real_') and file.endswith('.json'):
            test_files.append(f'/tmp/{file}')
    
    if not test_files:
        print("❌ No parity test data found. Run the parity test first.")
        return
    
    # Check for current test data first
    if os.path.exists('/tmp/parity_test_real_current.json'):
        latest_file = '/tmp/parity_test_real_current.json'
    else:
        # Use the most recent file
        latest_file = max(test_files, key=os.path.getctime)
    print(f"📂 Loading data from: {latest_file}")
    
    if not analyzer.load_real_data(latest_file):
        print("❌ Failed to load real hardware data")
        return
    
    # Analyze real hardware performance
    real_analysis = analyzer.analyze_real_performance()
    if not real_analysis:
        print("❌ Failed to analyze real hardware performance")
        return
    
    # Compare against expected simulation metrics
    comparison = analyzer.compare_metrics(real_analysis)
    if not comparison:
        print("❌ Failed to generate comparison metrics")
        return
    
    # Generate comprehensive report
    analyzer.generate_report(comparison, real_analysis)
    
    # Save analysis
    analyzer.save_analysis(comparison, real_analysis)
    
    print("🎯 Gazebo parity analysis complete!")


if __name__ == '__main__':
    main()