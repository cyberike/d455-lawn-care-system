#!/usr/bin/env python3
"""
Display D455 Lawn Care Evaluation Results
Shows performance summary and recommendations
"""

import json
import os
import sys
from datetime import datetime

def show_results(results_dir):
    """Display evaluation results from directory"""
    
    # Check if directory exists
    if not os.path.exists(results_dir):
        print(f"❌ Results directory not found: {results_dir}")
        return
    
    report_file = os.path.join(results_dir, 'performance_report.txt')
    data_file = os.path.join(results_dir, 'evaluation_data.json')
    plot_file = os.path.join(results_dir, 'performance_plots.png')
    
    print("🔍 D455 LAWN CARE EVALUATION RESULTS")
    print("=" * 50)
    
    # Display main report
    if os.path.exists(report_file):
        with open(report_file, 'r') as f:
            content = f.read()
            
        # Extract key metrics
        lines = content.split('\n')
        for line in lines:
            if 'Overall Score:' in line:
                score_part = line.split(':')[1].strip()
                score = float(score_part.split('/')[0])
                if '(Grade: ' in score_part:
                    grade = score_part.split('(Grade: ')[1].split(')')[0]
                else:
                    grade = 'N/A'
                
                if score >= 80:
                    emoji = "🟢"
                elif score >= 60:
                    emoji = "🟡"
                else:
                    emoji = "🔴"
                
                print(f"{emoji} Overall Performance: {score}/100 (Grade: {grade})")
                
            elif 'Grass Detection:' in line:
                score = line.split(':')[1].strip().split('/')[0]
                print(f"🌱 Grass Detection Score: {score}/100")
                
            elif 'Obstacle Detection:' in line:
                score = line.split(':')[1].strip().split('/')[0]
                print(f"🚧 Obstacle Detection Score: {score}/100")
    
    print()
    
    # Load detailed data
    if os.path.exists(data_file):
        with open(data_file, 'r') as f:
            data = json.load(f)
        
        # Grass detection details
        grass = data.get('grass_detection', {})
        print("📊 DETAILED METRICS")
        print("-" * 20)
        print(f"Grass Detection Rate: {grass.get('detection_rate', 0):.1f} Hz")
        print(f"Average Coverage: {grass.get('avg_coverage', 0):.2f}%")
        print(f"Messages Received: {grass.get('frame_count', 0)}")
        print(f"Coverage Consistency: {grass.get('consistency_score', 0):.1f}/100")
        
        # Obstacle detection details
        obstacle = data.get('obstacle_detection', {})
        print(f"\nObstacle Detection Rate: {obstacle.get('detection_rate', 0):.1f} Hz")
        print(f"Average Obstacles: {obstacle.get('avg_obstacles', 0):.1f}")
        print(f"Messages Received: {obstacle.get('frame_count', 0)}")
        
        # System performance
        system = data.get('system_performance', {})
        print(f"\nTotal Messages: {system.get('total_messages', 0)}")
        print(f"Evaluation Duration: {system.get('evaluation_duration', 0):.1f}s")
    
    print()
    
    # Show recommendations
    if os.path.exists(report_file):
        print("💡 TUNING RECOMMENDATIONS")
        print("-" * 25)
        
        with open(report_file, 'r') as f:
            content = f.read()
        
        # Extract recommendations section
        if "TUNING RECOMMENDATIONS" in content:
            recommendations_section = content.split("TUNING RECOMMENDATIONS")[1]
            lines = recommendations_section.split('\n')
            
            for line in lines[2:]:  # Skip header lines
                line = line.strip()
                if line.startswith('•'):
                    print(f"  {line}")
                elif not line:
                    continue
                elif line.startswith('-'):
                    break
    
    print()
    
    # File locations
    print("📁 FILES GENERATED")
    print("-" * 18)
    print(f"📄 Performance Report: {report_file}")
    print(f"📊 Raw Data (JSON): {data_file}")
    if os.path.exists(plot_file):
        print(f"📈 Visualizations: {plot_file}")
    
    print(f"\n📂 Results directory: {os.path.abspath(results_dir)}")


def main():
    if len(sys.argv) != 2:
        # Find most recent evaluation directory
        eval_dirs = [d for d in os.listdir('.') if d.startswith('evaluation_')]
        if eval_dirs:
            latest_dir = max(eval_dirs)
            print(f"Using latest evaluation: {latest_dir}")
            show_results(latest_dir)
        else:
            print("Usage: python3 show_evaluation_results.py <results_directory>")
            print("No evaluation directories found in current directory.")
            sys.exit(1)
    else:
        results_dir = sys.argv[1]
        show_results(results_dir)


if __name__ == '__main__':
    main()