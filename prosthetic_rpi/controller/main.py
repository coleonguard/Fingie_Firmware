#!/usr/bin/env python3
# Main application entry point for the prosthetic hand controller

import time
import signal
import sys
import argparse
import os
from pathlib import Path

# Import our controller module
from .hand_controller import HandController
from .data_replay import DataReplay
from .mujoco_model import HandModelGenerator
from .mujoco_replay import main as mujoco_replay_main

# Global controller reference for signal handler
controller = None

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down hand controller...")
    if controller:
        controller.stop()
    sys.exit(0)

def run_controller(args):
    """Run the prosthetic hand controller in real-time mode"""
    global controller
    
    print("Starting prosthetic hand controller...")
    
    # Create and start the hand controller
    controller = HandController(
        update_rate=args.rate, 
        enable_logging=not args.no_logging,
        enable_imu=not args.no_imu,
        log_path=args.log_path
    )
    controller.start()
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Main loop - keeps the program alive and allows for monitoring
        print("Controller running. Press Ctrl+C to exit.")
        print(f"Data logging: {'DISABLED' if args.no_logging else 'ENABLED'}")
        
        snapshot_interval = args.snapshot_interval
        last_snapshot_time = time.time()
        
        # Display information about finger state periodically
        while True:
            # Get state for all fingers
            fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
            
            print("\nCurrent finger status:")
            print("-" * 60)
            print(f"{'Finger':10} {'Distance':10} {'Mode':15} {'Current':10}")
            print("-" * 60)
            
            for finger in fingers:
                info = controller.get_finger_info(finger)
                print(f"{info['finger']:10} {info['distance']:10.1f} {info['mode']:15} {info['current']:10.3f}")
            
            # Save periodic snapshots if enabled
            if not args.no_logging and snapshot_interval > 0:
                current_time = time.time()
                if current_time - last_snapshot_time >= snapshot_interval:
                    controller.save_snapshot(f"auto_snapshot_{int(current_time)}")
                    last_snapshot_time = current_time
                    print("* Auto snapshot saved")
                
            # Wait before updating again
            time.sleep(args.interval)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Make sure we clean up properly
        controller.stop()

def replay_session(args):
    """Replay a recorded data session"""
    # First check if the session exists
    session_path = Path(args.session)
    if not session_path.exists() or not session_path.is_dir():
        # Try prepending the log path if it's not a full path
        session_path = Path(args.log_path) / args.session
        if not session_path.exists() or not session_path.is_dir():
            print(f"Error: Session directory not found: {args.session}")
            return 1
    
    print(f"Replaying session: {session_path}")
    
    # Create replay object
    replay = DataReplay(session_path)
    
    if args.plot:
        # If plotting is requested, generate plots for each finger
        fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        for finger in fingers:
            print(f"Generating plot for {finger}...")
            fig = replay.plot_finger_data(finger)
            if fig:
                plot_path = session_path / f"{finger}_plot.png"
                fig.savefig(plot_path)
                print(f"Plot saved to {plot_path}")
    
    if args.analyze:
        # Run transition analysis
        print("Analyzing phase transitions...")
        transitions = replay.analyze_phase_transitions()
        if transitions is not None:
            analysis_path = session_path / "phase_transitions.csv"
            transitions.to_csv(analysis_path)
            print(f"Transition analysis saved to {analysis_path}")
    
    if not args.no_replay:
        # Replay the session
        replay.replay_session(replay_factor=args.speed)
    
    return 0

def list_sessions(args):
    """List available data sessions"""
    replay = DataReplay()
    sessions = replay.get_available_sessions(args.log_path)
    
    if not sessions:
        print(f"No sessions found in {args.log_path}")
        return
    
    print(f"Available sessions in {args.log_path}:")
    print("-" * 60)
    for i, session in enumerate(sessions):
        # Try to get session metadata
        metadata_path = session / "metadata.json"
        session_time = session.name.replace("session_", "")
        
        # Count number of records
        record_count = "Unknown"
        json_path = session / "complete_dataset.json"
        csv_path = session / "sensor_data.csv"
        
        if json_path.exists():
            try:
                import json
                with open(json_path, 'r') as f:
                    data = json.load(f)
                record_count = len(data)
            except:
                pass
        elif csv_path.exists():
            try:
                with open(csv_path, 'r') as f:
                    record_count = sum(1 for _ in f) - 1  # Subtract header
            except:
                pass
        
        print(f"{i+1:2d}. {session.name} ({record_count} records)")
    
    print("-" * 60)

def create_mujoco_model(args):
    """Create a MuJoCo model for visualization and replay"""
    generator = HandModelGenerator(output_dir=args.output_dir)
    
    if args.type == "default":
        # Generate a basic hand model
        model_path = generator.generate_base_model()
        print(f"Basic hand model created: {model_path}")
        
    elif args.type == "relaxed":
        # Generate a model with the hand in relaxed pose
        model_path = generator.create_static_pose_model(pose_name="relaxed")
        print(f"Relaxed pose model created: {model_path}")
        
    elif args.type == "replay":
        # Generate a model for replaying the given data file
        if not args.data_file:
            print("Error: data-file is required for replay model type")
            return 1
            
        model_path = generator.create_data_replay_model(
            args.data_file, output_name=args.output_name)
            
        if model_path:
            print(f"Replay model created: {model_path}")
        else:
            print("Failed to create replay model")
            return 1
            
    else:
        print(f"Unknown model type: {args.type}")
        return 1
        
    # Visualize hand dimensions if requested
    if args.visualize:
        viz_path = generator.visualize_hand_dimensions()
        print(f"Hand visualization created: {viz_path}")
    
    return 0

def main():
    """Main entry point for the prosthetic hand controller"""
    # Default data directory
    default_log_path = os.path.expanduser("~/hand_data")
    default_model_path = os.path.expanduser("~/mujoco_models")
    
    # Create main parser
    parser = argparse.ArgumentParser(description="Prosthetic Hand Controller")
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Run controller command
    run_parser = subparsers.add_parser("run", help="Run the controller in real-time")
    run_parser.add_argument("--rate", type=int, default=20, help="Control rate in Hz (default: 20)")
    run_parser.add_argument("--interval", type=float, default=1.0, help="Status display interval in seconds (default: 1.0)")
    run_parser.add_argument("--no-logging", action="store_true", help="Disable data logging")
    run_parser.add_argument("--no-imu", action="store_true", help="Disable IMU tracking")
    run_parser.add_argument("--log-path", type=str, default=default_log_path, help=f"Path to store log data (default: {default_log_path})")
    run_parser.add_argument("--snapshot-interval", type=float, default=0, help="Auto-snapshot interval in seconds (0 to disable)")
    
    # Replay command
    replay_parser = subparsers.add_parser("replay", help="Replay a recorded session")
    replay_parser.add_argument("session", help="Session directory name or path")
    replay_parser.add_argument("--speed", type=float, default=1.0, help="Replay speed factor (default: 1.0)")
    replay_parser.add_argument("--no-replay", action="store_true", help="Skip actual replay (useful with --plot or --analyze)")
    replay_parser.add_argument("--plot", action="store_true", help="Generate plots of the session")
    replay_parser.add_argument("--analyze", action="store_true", help="Analyze phase transitions")
    replay_parser.add_argument("--log-path", type=str, default=default_log_path, help=f"Path to sessions if relative path given (default: {default_log_path})")
    
    # List sessions command
    list_parser = subparsers.add_parser("list", help="List available data sessions")
    list_parser.add_argument("--log-path", type=str, default=default_log_path, help=f"Path to sessions directory (default: {default_log_path})")
    
    # MuJoCo model command
    mujoco_parser = subparsers.add_parser("mujoco", help="Generate MuJoCo models for visualization and replay")
    mujoco_parser.add_argument("--type", type=str, choices=["default", "relaxed", "replay"], default="relaxed",
                             help="Type of model to generate (default: relaxed)")
    mujoco_parser.add_argument("--data-file", type=str, help="Data file to use for replay model (required for replay type)")
    mujoco_parser.add_argument("--output-dir", type=str, default=default_model_path, 
                             help=f"Directory to save models (default: {default_model_path})")
    mujoco_parser.add_argument("--output-name", type=str, default="replay_model",
                             help="Name for output model file (default: replay_model)")
    mujoco_parser.add_argument("--visualize", action="store_true", help="Generate hand dimension visualization")
    
    # Visualization commands
    viz_parser = subparsers.add_parser("visualize", help="Generate 2D hand visualizations")
    viz_subparsers = viz_parser.add_subparsers(dest="viz_command", help="Visualization command")
    
    # Static visualization
    static_parser = viz_subparsers.add_parser("static", help="Generate static hand visualization")
    static_parser.add_argument("--params", type=str, help="Path to custom parameters file")
    static_parser.add_argument("--output", type=str, default="hand_model_detailed.png", help="Output image file")
    
    # Animation from data
    anim_parser = viz_subparsers.add_parser("animate", help="Generate hand animation from recorded data")
    anim_parser.add_argument("data_file", help="Path to recorded data file (JSON or CSV)")
    anim_parser.add_argument("--output", type=str, default="hand_animation.mp4", help="Output video file")
    anim_parser.add_argument("--fps", type=int, default=30, help="Frames per second for animation (default: 30)")
    anim_parser.add_argument("--duration", type=float, help="Optional duration limit in seconds")
    
    # Parse args
    args = parser.parse_args()
    
    # Run appropriate command
    if args.command == "run":
        return run_controller(args)
    elif args.command == "replay":
        return replay_session(args)
    elif args.command == "list":
        return list_sessions(args)
    elif args.command == "mujoco":
        return create_mujoco_model(args)
    elif args.command == "visualize":
        # Handle visualization commands
        from .visualize_hand import draw_hand, generate_animation, load_params
        
        if args.viz_command == "static":
            # Generate static visualization
            params = load_params(args.params) if args.params else None
            draw_hand(params, output_file=args.output)
            print(f"Static hand visualization created: {args.output}")
            return 0
            
        elif args.viz_command == "animate":
            # Generate animation from data file
            try:
                output_path = generate_animation(
                    args.data_file, 
                    output_file=args.output,
                    fps=args.fps,
                    duration=args.duration
                )
                if output_path:
                    print(f"Hand movement animation created: {output_path}")
                    return 0
                else:
                    return 1
            except Exception as e:
                print(f"Error generating animation: {e}")
                return 1
        else:
            viz_parser.print_help()
            return 1
    else:
        # Default to run if no command specified
        parser.print_help()
        return 1

if __name__ == "__main__":
    sys.exit(main())
