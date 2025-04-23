#!/usr/bin/env python3
# Enhanced hand visualization script with configurable parameters

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Circle, Polygon, PathPatch
from matplotlib.path import Path
import matplotlib.transforms as transforms
import json
import os

# Default hand parameters - easily adjustable
DEFAULT_PARAMS = {
    "hand": {
        "palm_length": 0.09,     # 9cm palm length
        "palm_width": 0.08,      # 8cm palm width
        "palm_thickness": 0.03,  # 3cm palm thickness
        "wrist_width": 0.06,     # 6cm wrist width
        "wrist_length": 0.03,    # 3cm wrist length
        "is_right_hand": True    # Right hand configuration
    },
    "fingers": {
        "Thumb": {
            "has_basal": true,          # Thumb has CMC/basal joint
            "basal_length": 0.025,      # 2.5cm basal segment (metacarpal)
            "proximal_length": 0.035,   # 3.5cm proximal phalanx
            "distal_length": 0.025,     # 2.5cm distal phalanx
            "width": 0.018,             # 1.8cm width
            "base_pos": [-0.02, 0.035], # Position on palm
            "base_orientation": -10,    # Degrees rotation for CMC joint
            "cmc_angle": 35,            # CMC joint angle (degrees) - thumb sticking out
            "mcp_angle": 5,             # MCP joint angle (degrees) - minimal curl
            "ip_angle": 5               # IP joint angle (degrees) - minimal curl
        },
        "Index": {
            "proximal_length": 0.04,    # 4cm proximal phalanx
            "distal_length": 0.025,     # 2.5cm distal phalanx
            "width": 0.016,             # 1.6cm width
            "base_pos": [0.03, 0.03],   # Position on palm
            "base_orientation": 0,      # Degrees rotation
            "mcp_angle": 0,             # MCP joint angle (degrees)
            "pip_angle": 5              # PIP joint angle (degrees)
        },
        "Middle": {
            "proximal_length": 0.042,   # 4.2cm proximal phalanx
            "distal_length": 0.028,     # 2.8cm distal phalanx
            "width": 0.016,             # 1.6cm width
            "base_pos": [0.03, 0.01],   # Position on palm
            "base_orientation": 0,      # Degrees rotation
            "mcp_angle": 0,             # MCP joint angle (degrees)
            "pip_angle": 5              # PIP joint angle (degrees)
        },
        "Ring": {
            "proximal_length": 0.039,   # 3.9cm proximal phalanx
            "distal_length": 0.025,     # 2.5cm distal phalanx
            "width": 0.015,             # 1.5cm width
            "base_pos": [0.03, -0.01],  # Position on palm
            "base_orientation": 0,      # Degrees rotation
            "mcp_angle": 0,             # MCP joint angle (degrees)
            "pip_angle": 5              # PIP joint angle (degrees)
        },
        "Pinky": {
            "proximal_length": 0.035,   # 3.5cm proximal phalanx
            "distal_length": 0.02,      # 2.0cm distal phalanx
            "width": 0.014,             # 1.4cm width
            "base_pos": [0.03, -0.03],  # Position on palm
            "base_orientation": 0,      # Degrees rotation
            "mcp_angle": 0,             # MCP joint angle (degrees)
            "pip_angle": 5              # PIP joint angle (degrees)
        }
    },
    "sensors": {
        # Proximity sensors positioned on finger pulp, not directly on joints
        "prox_sensors": {
            "Thumb1": {
                "finger": "Thumb",
                "segment": "proximal",  # on proximal phalanx
                "position": 0.5,        # fraction along segment (0-1)
                "offset": 0.008         # offset from centerline
            },
            "Thumb2": {
                "finger": "Thumb",
                "segment": "distal",
                "position": 0.5,
                "offset": 0.008
            },
            "Index1": {
                "finger": "Index",
                "segment": "proximal",
                "position": 0.5,
                "offset": 0.008
            },
            "Index2": {
                "finger": "Index",
                "segment": "distal",
                "position": 0.5,
                "offset": 0.008
            },
            "Middle1": {
                "finger": "Middle",
                "segment": "proximal",
                "position": 0.5,
                "offset": 0.008
            },
            "Middle2": {
                "finger": "Middle",
                "segment": "distal",
                "position": 0.5,
                "offset": 0.008
            },
            "Ring1": {
                "finger": "Ring",
                "segment": "proximal",
                "position": 0.5,
                "offset": 0.008
            },
            "Ring2": {
                "finger": "Ring",
                "segment": "distal",
                "position": 0.5,
                "offset": 0.008
            },
            "Pinky1": {
                "finger": "Pinky",
                "segment": "proximal",
                "position": 0.5,
                "offset": 0.008
            },
            "Pinky2": {
                "finger": "Pinky",
                "segment": "distal",
                "position": 0.5,
                "offset": 0.008
            }
        },
        # IMU sensors
        "imu_sensors": {
            "back": {
                "position": [0.0, 0.0],
                "size": 0.015
            },
            "wrist": {
                "position": [-0.065, 0.0],
                "size": 0.015
            }
        }
    },
    "control": {
        "approach_threshold": 30,  # mm
        "contact_threshold": 5,    # mm
        "max_angle_mcp": 90,       # degrees
        "max_angle_pip": 100       # degrees
    }
}

def save_default_params(filepath="hand_parameters.json"):
    """Save default parameters to a JSON file for easy editing"""
    with open(filepath, 'w') as f:
        json.dump(DEFAULT_PARAMS, f, indent=2)
    print(f"Default parameters saved to {filepath}")

def load_params(filepath="hand_parameters.json"):
    """Load parameters from a JSON file"""
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            return json.load(f)
    else:
        print(f"Parameter file {filepath} not found, using defaults")
        return DEFAULT_PARAMS

def draw_hand(params=None, output_file="hand_model_detailed.png"):
    """Create a detailed visualization of the hand model using the provided parameters"""
    # Use provided parameters or load defaults
    if params is None:
        params = load_params()
        
    fig, ax = plt.subplots(figsize=(12, 14))
    
    # Extract parameters
    hand_params = params["hand"]
    finger_params = params["fingers"]
    sensor_params = params["sensors"]
    control_params = params["control"]
    
    # Determine if right or left hand and adjust accordingly
    is_right_hand = hand_params.get("is_right_hand", True)
    if is_right_hand:
        # Right hand - positive x is outward from palm
        mirror_factor = 1
    else:
        # Left hand - negative x is outward from palm
        mirror_factor = -1
    
    # Extract palm dimensions
    palm_length = hand_params["palm_length"]
    palm_width = hand_params["palm_width"]
    
    # Draw palm
    palm_rect = Rectangle((-palm_length/2 * mirror_factor, -palm_width/2), 
                         palm_length, palm_width, 
                         facecolor='lightgray', edgecolor='black', alpha=0.7)
    ax.add_patch(palm_rect)
    
    # Add wrist
    wrist_width = hand_params["wrist_width"]
    wrist_length = hand_params["wrist_length"]
    wrist_rect = Rectangle(((-palm_length/2 - wrist_length) * mirror_factor, -wrist_width/2), 
                          wrist_length, wrist_width, 
                          facecolor='lightblue', edgecolor='black', alpha=0.5)
    ax.add_patch(wrist_rect)
    
    # Draw IMU sensors
    for imu_name, imu_data in sensor_params["imu_sensors"].items():
        pos = imu_data["position"]
        # Mirror x coordinate for right/left hand
        imu_pos = [pos[0] * mirror_factor, pos[1]]
        imu_size = imu_data["size"]
        
        imu_circle = Circle(imu_pos, radius=imu_size, facecolor='blue', edgecolor='darkblue', alpha=0.7)
        ax.add_patch(imu_circle)
        plt.text(imu_pos[0], imu_pos[1], imu_name, ha='center', va='center', fontsize=8, color='white', weight='bold')
    
    # Dictionary to store computed joint positions for proximity sensor placement
    joint_positions = {}
    segment_lines = {}
    
    # Draw fingers with realistic shape
    for finger_name, params in finger_params.items():
        # Get base position and orientation
        base_pos_orig = params["base_pos"]
        # Mirror x coordinate for right/left hand
        base_pos = [base_pos_orig[0] * mirror_factor, base_pos_orig[1]]
        base_angle_deg = params["base_orientation"] * mirror_factor
        base_angle = np.radians(base_angle_deg)
        
        # Store MCP joint position
        joint_positions[f"{finger_name}_mcp"] = base_pos
        
        # Draw MCP joint
        mcp_joint = Circle(base_pos, radius=0.008, facecolor='red', edgecolor='darkred', alpha=0.7)
        ax.add_patch(mcp_joint)
        
        # Get joint angles
        if finger_name == "Thumb":
            mcp_angle_deg = params["mcp_angle"]
            pip_angle_deg = params["ip_angle"]
        else:
            mcp_angle_deg = params["mcp_angle"]
            pip_angle_deg = params["pip_angle"]
        
        # Calculate proximal phalanx position
        prox_length = params["proximal_length"]
        mcp_angle = np.radians(base_angle_deg + mcp_angle_deg * mirror_factor)
        
        pip_pos = [
            base_pos[0] + prox_length * np.cos(mcp_angle),
            base_pos[1] + prox_length * np.sin(mcp_angle)
        ]
        
        # Store PIP joint position
        joint_positions[f"{finger_name}_pip"] = pip_pos
        
        # Store segment line for sensor positioning
        segment_lines[f"{finger_name}_proximal"] = (base_pos, pip_pos, mcp_angle)
        
        # Draw proximal phalanx as a realistic rounded rectangle
        finger_width = params["width"]
        draw_finger_segment(ax, base_pos, pip_pos, width=finger_width, angle=mcp_angle)
        
        # Draw PIP joint
        pip_joint = Circle(pip_pos, radius=0.006, facecolor='green', edgecolor='darkgreen', alpha=0.7)
        ax.add_patch(pip_joint)
        
        # Calculate distal phalanx position
        dist_length = params["distal_length"]
        pip_angle = np.radians(base_angle_deg + mcp_angle_deg * mirror_factor + pip_angle_deg * mirror_factor)
        
        tip_pos = [
            pip_pos[0] + dist_length * np.cos(pip_angle),
            pip_pos[1] + dist_length * np.sin(pip_angle)
        ]
        
        # Store fingertip position
        joint_positions[f"{finger_name}_tip"] = tip_pos
        
        # Store segment line for sensor positioning
        segment_lines[f"{finger_name}_distal"] = (pip_pos, tip_pos, pip_angle)
        
        # Draw distal phalanx
        draw_finger_segment(ax, pip_pos, tip_pos, width=finger_width*0.9, angle=pip_angle)
        
        # Label finger
        lbl_offset_x = 0.01 * mirror_factor
        plt.text(tip_pos[0] + lbl_offset_x, tip_pos[1], finger_name, 
                 fontsize=10, ha='left' if mirror_factor > 0 else 'right')
        
        # Add joint labels - make them smaller and position better
        mcp_label = "MCP"
        if finger_name == "Thumb":
            pip_label = "IP"
        else:
            pip_label = "PIP"
            
        # Position labels at slightly offset positions
        label_offset = 0.005 * mirror_factor
        plt.text(base_pos[0] - label_offset, base_pos[1] - 0.005, 
                 mcp_label, fontsize=6, ha='right' if mirror_factor > 0 else 'left')
        plt.text(pip_pos[0] - label_offset, pip_pos[1] - 0.005, 
                 pip_label, fontsize=6, ha='right' if mirror_factor > 0 else 'left')
    
    # Add proximity sensors - positioned on finger pulp, not directly on joints
    for sensor_name, sensor_data in sensor_params["prox_sensors"].items():
        finger = sensor_data["finger"]
        segment = sensor_data["segment"]
        pos_fraction = sensor_data["position"]  # 0-1 along segment
        offset = sensor_data["offset"]  # offset from centerline
        
        # Get segment line
        segment_key = f"{finger}_{segment}"
        if segment_key in segment_lines:
            start_pos, end_pos, angle = segment_lines[segment_key]
            
            # Interpolate position along segment
            sensor_x = start_pos[0] + (end_pos[0] - start_pos[0]) * pos_fraction
            sensor_y = start_pos[1] + (end_pos[1] - start_pos[1]) * pos_fraction
            
            # Add offset perpendicular to segment
            perp_angle = angle + np.pi/2
            sensor_x += offset * np.cos(perp_angle)
            sensor_y += offset * np.sin(perp_angle)
            
            # Draw sensor
            sensor_circle = Circle((sensor_x, sensor_y), radius=0.005, 
                                  facecolor='yellow', edgecolor='orange', alpha=0.9)
            ax.add_patch(sensor_circle)
            
            # Add sensor label in small font
            plt.text(sensor_x, sensor_y - 0.005, sensor_name, 
                    fontsize=6, ha='center', va='top')
    
    # Set plot properties - accommodate the right hand configuration
    if is_right_hand:
        x_min, x_max = -0.1, 0.1
    else:
        x_min, x_max = -0.1, 0.1
        
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-0.1, 0.1)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # Title indicating right or left hand
    hand_type = "Right Hand" if is_right_hand else "Left Hand"
    ax.set_title(f'Prosthetic {hand_type} Model - Relaxed Pose (Top View)')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    
    # Add legend
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    
    legend_elements = [
        Patch(facecolor='lightgray', edgecolor='black', alpha=0.7, label='Palm'),
        Patch(facecolor='lightblue', edgecolor='black', alpha=0.5, label='Wrist'),
        Patch(facecolor='red', edgecolor='darkred', alpha=0.7, label='MCP Joints'),
        Patch(facecolor='green', edgecolor='darkgreen', alpha=0.7, label='PIP/IP Joints'),
        Patch(facecolor='blue', edgecolor='darkblue', alpha=0.7, label='IMU Sensors'),
        Patch(facecolor='yellow', edgecolor='orange', alpha=0.9, label='Proximity Sensors')
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    # Add annotations for control parameters
    plt.annotate(
        f"Control Parameters:\n"
        f"- >{control_params['approach_threshold']}mm: Approach (No movement)\n"
        f"- {control_params['contact_threshold']}-{control_params['approach_threshold']}mm: Proportional Control\n"
        f"- <{control_params['contact_threshold']}mm: Contact (Torque Control)",
        xy=(-0.09, -0.07), xytext=(-0.09, -0.07),
        bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.8)
    )
    
    # Save visualization
    plt.savefig(output_file, dpi=300)
    print(f"Detailed hand visualization saved to: {output_file}")
    plt.close()
    
    return output_file

def draw_finger_segment(ax, start_pos, end_pos, width, angle):
    """Draw a finger segment as a realistic rounded rectangle"""
    # Calculate length of segment
    length = np.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
    
    # Create a rectangle centered at origin
    rect_width = width
    rect_length = length
    
    # Define the corners of the rectangle
    rect_corners = np.array([
        [-rect_length/2, -rect_width/2],
        [rect_length/2, -rect_width/2],
        [rect_length/2, rect_width/2],
        [-rect_length/2, rect_width/2]
    ])
    
    # Create rotation matrix
    rotation = transforms.Affine2D().rotate(angle)
    
    # Create translation matrix to move to segment midpoint
    mid_x = (start_pos[0] + end_pos[0]) / 2
    mid_y = (start_pos[1] + end_pos[1]) / 2
    translation = transforms.Affine2D().translate(mid_x, mid_y)
    
    # Apply transformations
    rect_corners = rotation.transform(rect_corners)
    rect_corners = translation.transform(rect_corners)
    
    # Create polygon patch
    polygon = Polygon(rect_corners, closed=True, 
                      facecolor='bisque', edgecolor='saddlebrown', alpha=0.8)
    ax.add_patch(polygon)
    
    # Add rounded ends (circles at each end)
    start_circle = Circle(start_pos, radius=width/2, 
                         facecolor='bisque', edgecolor='saddlebrown', alpha=0.8)
    end_circle = Circle(end_pos, radius=width/2, 
                       facecolor='bisque', edgecolor='saddlebrown', alpha=0.8)
    ax.add_patch(start_circle)
    ax.add_patch(end_circle)

def generate_animation(data_file, output_file="hand_animation.mp4", fps=30, duration=None):
    """Generate an animation of hand movement based on recorded data
    
    Args:
        data_file: Path to the recorded data file (JSON or CSV)
        output_file: Path for the output video file
        fps: Frames per second for the video
        duration: Optional duration limit in seconds
        
    Returns:
        Path to the generated video file
    """
    import pandas as pd
    import json
    import matplotlib.animation as animation
    from matplotlib.animation import FFMpegWriter
    import numpy as np
    from pathlib import Path
    
    # Load the data
    try:
        # Try loading as JSON
        try:
            with open(data_file, 'r') as f:
                data = json.load(f)
            df = pd.DataFrame(data)
        except:
            # Try loading as CSV
            df = pd.read_csv(data_file)
            
        print(f"Loaded {len(df)} records from {data_file}")
    except Exception as e:
        print(f"Error loading data file: {e}")
        return None
    
    # Sample the data to match the desired FPS
    # If we have timestamps, use those for accurate timing
    if 'timestamp' in df.columns:
        # Convert timestamps to relative time in seconds
        df['time_seconds'] = df['timestamp'] - df['timestamp'].iloc[0]
        total_time = df['time_seconds'].iloc[-1]
    else:
        # Create equally spaced time steps
        df['time_seconds'] = np.linspace(0, len(df)/10, len(df))  # Assume 10Hz if no timestamps
        total_time = len(df)/10
    
    print(f"Data spans {total_time:.2f} seconds")
    
    # If duration is provided, limit the data
    if duration and duration < total_time:
        df = df[df['time_seconds'] <= duration]
        total_time = duration
        print(f"Limited to first {duration:.2f} seconds")
    
    # Calculate how many frames to generate
    n_frames = int(fps * total_time)
    print(f"Generating {n_frames} frames at {fps} fps")
    
    # Create time points for each frame
    frame_times = np.linspace(0, total_time, n_frames)
    
    # Load default hand parameters
    base_params = load_params()
    
    # Create figure for animation
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Initialize progress tracking
    frame_count = 0
    total_frames = len(frame_times)
    
    # Function to update the plot for each frame
    def update(frame_idx):
        nonlocal frame_count
        
        # Clear the current plot
        ax.clear()
        
        # Get the time for this frame
        frame_time = frame_times[frame_idx]
        
        # Find the closest data point in time
        closest_idx = (df['time_seconds'] - frame_time).abs().idxmin()
        frame_data = df.iloc[closest_idx]
        
        # Create a deep copy of the base parameters
        params = json.loads(json.dumps(base_params))
        
        # Update finger positions based on sensor data
        for finger in ["Thumb", "Index", "Middle", "Ring", "Pinky"]:
            # Check if we have filtered proximity sensor data
            sensor_key = f"{finger}1_filtered"
            if sensor_key in frame_data:
                distance = frame_data[sensor_key]
                
                # Convert distance to joint angles
                # Start with basic mapping: longer distance = less bend
                approach_threshold = params["control"]["approach_threshold"]
                contact_threshold = params["control"]["contact_threshold"]
                
                # Determine MCP and PIP/IP angles based on distance
                if distance >= approach_threshold:
                    # Approach phase - no bending
                    mcp_angle = 0
                    pip_angle = 0
                elif distance <= contact_threshold:
                    # Contact phase - maximum bending
                    mcp_angle = params["control"]["max_angle_mcp"]
                    pip_angle = params["control"]["max_angle_pip"]
                else:
                    # Proportional phase
                    normalized = (approach_threshold - distance) / (approach_threshold - contact_threshold)
                    mcp_angle = normalized * params["control"]["max_angle_mcp"]
                    pip_angle = normalized * params["control"]["max_angle_pip"]
                
                # Update the parameters
                if finger == "Thumb":
                    params["fingers"][finger]["mcp_angle"] = mcp_angle
                    params["fingers"][finger]["ip_angle"] = pip_angle
                else:
                    params["fingers"][finger]["mcp_angle"] = mcp_angle
                    params["fingers"][finger]["pip_angle"] = pip_angle
        
        # Update hand orientation if IMU data is available
        if all(k in frame_data for k in ["back_imu_roll", "back_imu_pitch", "back_imu_yaw"]):
            # Apply a global rotation to the entire hand based on IMU data
            # This would be a more complex transformation in a real 3D environment
            # For our 2D visualization, we'll just note it in the title
            roll = frame_data["back_imu_roll"]
            pitch = frame_data["back_imu_pitch"]
            yaw = frame_data["back_imu_yaw"]
            orientation_info = f"IMU: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°"
        else:
            orientation_info = "No IMU data"
        
        # Draw the hand with updated parameters
        draw_hand(params, output_file=None, ax=ax)
        
        # Add frame information
        ax.set_title(f'Hand Animation - Time: {frame_time:.2f}s\n{orientation_info}')
        
        # Update progress
        frame_count += 1
        if frame_count % 10 == 0 or frame_count == total_frames:
            print(f"Progress: {frame_count}/{total_frames} frames ({frame_count/total_frames*100:.1f}%)")
        
        return ax,
    
    # Create the animation
    print("Creating animation...")
    anim = animation.FuncAnimation(fig, update, frames=len(frame_times), blit=False)
    
    # Save the animation
    writer = FFMpegWriter(fps=fps, metadata=dict(artist='HandVisualizer'), bitrate=5000)
    anim.save(output_file, writer=writer)
    
    print(f"Animation saved to: {output_file}")
    plt.close()
    
    return output_file

def draw_hand(params=None, output_file="hand_model_detailed.png", ax=None):
    """Create a detailed visualization of the hand model using the provided parameters
    
    Args:
        params: Hand parameters dictionary
        output_file: Path to save the output image (None to skip saving)
        ax: Optional matplotlib axes to draw on
        
    Returns:
        Path to the output file or the axes object
    """
    # Use provided parameters or load defaults
    if params is None:
        params = load_params()
    
    # Create figure if not provided
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 14))
        created_figure = True
    else:
        created_figure = False
    
    # Extract parameters
    hand_params = params["hand"]
    finger_params = params["fingers"]
    sensor_params = params["sensors"]
    control_params = params["control"]
    
    # Determine if right or left hand and adjust accordingly
    is_right_hand = hand_params.get("is_right_hand", True)
    if is_right_hand:
        # Right hand - positive x is outward from palm
        mirror_factor = 1
    else:
        # Left hand - negative x is outward from palm
        mirror_factor = -1
    
    # Extract palm dimensions
    palm_length = hand_params["palm_length"]
    palm_width = hand_params["palm_width"]
    
    # Draw palm
    palm_rect = Rectangle((-palm_length/2 * mirror_factor, -palm_width/2), 
                         palm_length, palm_width, 
                         facecolor='lightgray', edgecolor='black', alpha=0.7)
    ax.add_patch(palm_rect)
    
    # Add wrist
    wrist_width = hand_params["wrist_width"]
    wrist_length = hand_params["wrist_length"]
    wrist_rect = Rectangle(((-palm_length/2 - wrist_length) * mirror_factor, -wrist_width/2), 
                          wrist_length, wrist_width, 
                          facecolor='lightblue', edgecolor='black', alpha=0.5)
    ax.add_patch(wrist_rect)
    
    # Draw IMU sensors
    for imu_name, imu_data in sensor_params["imu_sensors"].items():
        pos = imu_data["position"]
        # Mirror x coordinate for right/left hand
        imu_pos = [pos[0] * mirror_factor, pos[1]]
        imu_size = imu_data["size"]
        
        imu_circle = Circle(imu_pos, radius=imu_size, facecolor='blue', edgecolor='darkblue', alpha=0.7)
        ax.add_patch(imu_circle)
        plt.text(imu_pos[0], imu_pos[1], imu_name, ha='center', va='center', fontsize=8, color='white', weight='bold')
    
    # Dictionary to store computed joint positions for proximity sensor placement
    joint_positions = {}
    segment_lines = {}
    
    # Draw fingers with realistic shape
    for finger_name, params in finger_params.items():
        # Get base position and orientation
        base_pos_orig = params["base_pos"]
        # Mirror x coordinate for right/left hand
        base_pos = [base_pos_orig[0] * mirror_factor, base_pos_orig[1]]
        base_angle_deg = params["base_orientation"] * mirror_factor
        base_angle = np.radians(base_angle_deg)
        
        # Store MCP joint position
        joint_positions[f"{finger_name}_mcp"] = base_pos
        
        # Draw MCP joint
        mcp_joint = Circle(base_pos, radius=0.008, facecolor='red', edgecolor='darkred', alpha=0.7)
        ax.add_patch(mcp_joint)
        
        # Get joint angles
        if finger_name == "Thumb":
            mcp_angle_deg = params["mcp_angle"]
            pip_angle_deg = params["ip_angle"]
        else:
            mcp_angle_deg = params["mcp_angle"]
            pip_angle_deg = params["pip_angle"]
        
        # Calculate proximal phalanx position
        prox_length = params["proximal_length"]
        mcp_angle = np.radians(base_angle_deg + mcp_angle_deg * mirror_factor)
        
        pip_pos = [
            base_pos[0] + prox_length * np.cos(mcp_angle),
            base_pos[1] + prox_length * np.sin(mcp_angle)
        ]
        
        # Store PIP joint position
        joint_positions[f"{finger_name}_pip"] = pip_pos
        
        # Store segment line for sensor positioning
        segment_lines[f"{finger_name}_proximal"] = (base_pos, pip_pos, mcp_angle)
        
        # Draw proximal phalanx as a realistic rounded rectangle
        finger_width = params["width"]
        draw_finger_segment(ax, base_pos, pip_pos, width=finger_width, angle=mcp_angle)
        
        # Draw PIP joint
        pip_joint = Circle(pip_pos, radius=0.006, facecolor='green', edgecolor='darkgreen', alpha=0.7)
        ax.add_patch(pip_joint)
        
        # Calculate distal phalanx position
        dist_length = params["distal_length"]
        pip_angle = np.radians(base_angle_deg + mcp_angle_deg * mirror_factor + pip_angle_deg * mirror_factor)
        
        tip_pos = [
            pip_pos[0] + dist_length * np.cos(pip_angle),
            pip_pos[1] + dist_length * np.sin(pip_angle)
        ]
        
        # Store fingertip position
        joint_positions[f"{finger_name}_tip"] = tip_pos
        
        # Store segment line for sensor positioning
        segment_lines[f"{finger_name}_distal"] = (pip_pos, tip_pos, pip_angle)
        
        # Draw distal phalanx
        draw_finger_segment(ax, pip_pos, tip_pos, width=finger_width*0.9, angle=pip_angle)
        
        # Label finger
        lbl_offset_x = 0.01 * mirror_factor
        plt.text(tip_pos[0] + lbl_offset_x, tip_pos[1], finger_name, 
                 fontsize=10, ha='left' if mirror_factor > 0 else 'right')
        
        # Add joint labels - make them smaller and position better
        mcp_label = "MCP"
        if finger_name == "Thumb":
            pip_label = "IP"
        else:
            pip_label = "PIP"
            
        # Position labels at slightly offset positions
        label_offset = 0.005 * mirror_factor
        plt.text(base_pos[0] - label_offset, base_pos[1] - 0.005, 
                 mcp_label, fontsize=6, ha='right' if mirror_factor > 0 else 'left')
        plt.text(pip_pos[0] - label_offset, pip_pos[1] - 0.005, 
                 pip_label, fontsize=6, ha='right' if mirror_factor > 0 else 'left')
    
    # Add proximity sensors - positioned on finger pulp, not directly on joints
    for sensor_name, sensor_data in sensor_params["prox_sensors"].items():
        finger = sensor_data["finger"]
        segment = sensor_data["segment"]
        pos_fraction = sensor_data["position"]  # 0-1 along segment
        offset = sensor_data["offset"]  # offset from centerline
        
        # Get segment line
        segment_key = f"{finger}_{segment}"
        if segment_key in segment_lines:
            start_pos, end_pos, angle = segment_lines[segment_key]
            
            # Interpolate position along segment
            sensor_x = start_pos[0] + (end_pos[0] - start_pos[0]) * pos_fraction
            sensor_y = start_pos[1] + (end_pos[1] - start_pos[1]) * pos_fraction
            
            # Add offset perpendicular to segment
            perp_angle = angle + np.pi/2
            sensor_x += offset * np.cos(perp_angle)
            sensor_y += offset * np.sin(perp_angle)
            
            # Draw sensor
            sensor_circle = Circle((sensor_x, sensor_y), radius=0.005, 
                                  facecolor='yellow', edgecolor='orange', alpha=0.9)
            ax.add_patch(sensor_circle)
            
            # Add sensor label in small font
            plt.text(sensor_x, sensor_y - 0.005, sensor_name, 
                    fontsize=6, ha='center', va='top')
    
    # Set plot properties - accommodate the right hand configuration
    if is_right_hand:
        x_min, x_max = -0.1, 0.1
    else:
        x_min, x_max = -0.1, 0.1
        
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-0.1, 0.1)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.7)
    
    # Title indicating right or left hand
    hand_type = "Right Hand" if is_right_hand else "Left Hand"
    ax.set_title(f'Prosthetic {hand_type} Model - Relaxed Pose (Top View)')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    
    # Add legend
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    
    legend_elements = [
        Patch(facecolor='lightgray', edgecolor='black', alpha=0.7, label='Palm'),
        Patch(facecolor='lightblue', edgecolor='black', alpha=0.5, label='Wrist'),
        Patch(facecolor='red', edgecolor='darkred', alpha=0.7, label='MCP Joints'),
        Patch(facecolor='green', edgecolor='darkgreen', alpha=0.7, label='PIP/IP Joints'),
        Patch(facecolor='blue', edgecolor='darkblue', alpha=0.7, label='IMU Sensors'),
        Patch(facecolor='yellow', edgecolor='orange', alpha=0.9, label='Proximity Sensors')
    ]
    ax.legend(handles=legend_elements, loc='upper right')
    
    # Add annotations for control parameters
    plt.annotate(
        f"Control Parameters:\n"
        f"- >{control_params['approach_threshold']}mm: Approach (No movement)\n"
        f"- {control_params['contact_threshold']}-{control_params['approach_threshold']}mm: Proportional Control\n"
        f"- <{control_params['contact_threshold']}mm: Contact (Torque Control)",
        xy=(-0.09, -0.07), xytext=(-0.09, -0.07),
        bbox=dict(boxstyle="round,pad=0.5", facecolor='white', alpha=0.8)
    )
    
    # Save visualization if output file is specified
    if output_file and created_figure:
        plt.savefig(output_file, dpi=300)
        print(f"Detailed hand visualization saved to: {output_file}")
        plt.close()
        return output_file
    
    # Return the axes object if we didn't create a new figure
    if not created_figure:
        return ax

if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Hand visualization and animation tools")
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Static visualization command
    static_parser = subparsers.add_parser("static", help="Generate static hand visualization")
    static_parser.add_argument("--params", help="Path to custom parameters file")
    static_parser.add_argument("--output", default="hand_model_detailed.png", help="Output image file")
    
    # Animation command
    anim_parser = subparsers.add_parser("animate", help="Generate hand animation from data")
    anim_parser.add_argument("data_file", help="Path to data file (JSON or CSV)")
    anim_parser.add_argument("--output", default="hand_animation.mp4", help="Output video file")
    anim_parser.add_argument("--fps", type=int, default=30, help="Frames per second")
    anim_parser.add_argument("--duration", type=float, help="Duration limit in seconds")
    
    # Generate parameters command
    params_parser = subparsers.add_parser("params", help="Generate default parameters file")
    params_parser.add_argument("--output", default="hand_parameters.json", help="Output parameter file")
    
    args = parser.parse_args()
    
    # Execute the appropriate command
    if args.command == "static":
        # Generate static visualization
        params = load_params(args.params) if args.params else None
        draw_hand(params, output_file=args.output)
        
    elif args.command == "animate":
        # Generate animation
        generate_animation(args.data_file, output_file=args.output, fps=args.fps, duration=args.duration)
        
    elif args.command == "params":
        # Generate default parameters file
        save_default_params(args.output)
        
    else:
        # Default behavior - generate static visualization with default parameters
        save_default_params()
        draw_hand()