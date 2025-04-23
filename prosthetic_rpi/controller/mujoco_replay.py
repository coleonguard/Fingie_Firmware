#!/usr/bin/env python3
# MuJoCo Replay Tool: Converts logged data to MuJoCo animation

import argparse
import os
import sys
from pathlib import Path

from .mujoco_model import HandModelGenerator
from .data_replay import DataReplay

def main():
    """Main function for MuJoCo data replay tool"""
    # Set up argument parser
    parser = argparse.ArgumentParser(description="MuJoCo Replay Tool for Prosthetic Hand Data")
    
    # Add commands
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Generate default models command
    gen_parser = subparsers.add_parser("generate", help="Generate default hand models")
    gen_parser.add_argument("--output-dir", type=str, default="mujoco_models", 
                           help="Directory to save models (default: mujoco_models)")
    
    # Create replay model command
    replay_parser = subparsers.add_parser("replay", help="Create replay model from data")
    replay_parser.add_argument("data_file", help="Path to data file (JSON or CSV)")
    replay_parser.add_argument("--output-name", type=str, default="replay_model",
                              help="Name for output model file (default: replay_model)")
    replay_parser.add_argument("--output-dir", type=str, default="mujoco_models",
                              help="Directory to save model (default: mujoco_models)")
    
    # Visualization command
    viz_parser = subparsers.add_parser("visualize", help="Visualize hand dimensions")
    viz_parser.add_argument("--output-dir", type=str, default="mujoco_models",
                           help="Directory to save visualization (default: mujoco_models)")
    
    # Parse arguments
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    if hasattr(args, 'output_dir'):
        output_dir = Path(args.output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
    
    # Execute command
    if args.command == "generate":
        # Generate default models
        generator = HandModelGenerator(output_dir=args.output_dir)
        base_model_path = generator.generate_base_model()
        relaxed_model_path = generator.create_static_pose_model(pose_name="relaxed")
        
        print(f"\nModels generated successfully:")
        print(f"- Base model: {base_model_path}")
        print(f"- Relaxed pose model: {relaxed_model_path}")
        
    elif args.command == "replay":
        # Create replay model from data
        generator = HandModelGenerator(output_dir=args.output_dir)
        replay_model_path = generator.create_data_replay_model(
            args.data_file, output_name=args.output_name)
        
        if replay_model_path:
            print(f"\nReplay model generated successfully:")
            print(f"- Replay model: {replay_model_path}")
            print("\nTo view this model with MuJoCo:")
            print(f"python -c \"import mujoco_py; model = mujoco_py.load_model_from_path('{replay_model_path}'); sim = mujoco_py.MjSim(model); viewer = mujoco_py.MjViewer(sim); viewer.render()\"")
        else:
            print("Failed to generate replay model.")
            return 1
            
    elif args.command == "visualize":
        # Visualize hand dimensions
        generator = HandModelGenerator(output_dir=args.output_dir)
        viz_path = generator.visualize_hand_dimensions()
        
        print(f"\nHand visualization generated successfully:")
        print(f"- Visualization: {viz_path}")
        
    else:
        # Display help if no command specified
        parser.print_help()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())