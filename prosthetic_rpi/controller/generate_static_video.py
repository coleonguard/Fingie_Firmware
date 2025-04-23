#!/usr/bin/env python3
# Generate a static pose video with proximity sensor heat map visualization

import sys
import os

# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# Add parent directory to path (needed for direct execution)
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, parent_dir)

# Import the improved visualization module
from visualize_hand_improved import generate_static_video

# Create visualization output directory
output_dir = os.path.join(os.path.dirname(os.path.dirname(
               os.path.dirname(os.path.abspath(__file__)))), "visualizations")
os.makedirs(output_dir, exist_ok=True)
output_file = os.path.join(output_dir, "hand_static_with_proximity.mp4")

# Create mock proximity values ranging from near to far
mock_prox_values = {
    "Thumb1": 10,  # Close proximity
    "Thumb2": 5,   # Contact
    "Index1": 15,
    "Index2": 20,
    "Middle1": 25, 
    "Middle2": 30,  # Approach threshold
    "Ring1": 35,
    "Ring2": 40,   # Beyond approach threshold
    "Pinky1": 50,  # Far
    "Pinky2": 45
}

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Generate static hand pose video with proximity sensor heatmap")
    parser.add_argument("--output", default=output_file, help="Output video file")
    parser.add_argument("--duration", type=float, default=2.0, help="Video duration in seconds")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    parser.add_argument("--params", help="Custom parameters file")
    
    args = parser.parse_args()
    
    # Generate the video with mock proximity values
    generate_static_video(
        output_file=args.output,
        duration=args.duration,
        fps=args.fps,
        params_file=args.params,
        mock_prox_values=mock_prox_values
    )
    
    print(f"Generated video with proximity heatmap visualization: {args.output}")