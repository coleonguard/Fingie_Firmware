# MuJoCo Model Generator: Creates hand models for visualization and replay

import os
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from pathlib import Path

class HandModelGenerator:
    """Generates MuJoCo XML model of the prosthetic hand based on parameters
    
    This module creates:
    1. Parameterized XML model of the hand with appropriate joints
    2. Default relaxed position visualization
    3. Integration with recorded data for replay
    """
    
    def __init__(self, output_dir="mujoco_models"):
        """Initialize the hand model generator
        
        Args:
            output_dir: Directory to save generated models
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True, parents=True)
        
        # Default hand parameters (in meters)
        self.palm_length = 0.09  # 9cm palm length
        self.palm_width = 0.08   # 8cm palm width
        self.palm_height = 0.03  # 3cm palm thickness
        
        # Finger dimensions (in meters)
        self.finger_params = {
            "Thumb": {
                "proximal_length": 0.04,   # 4cm proximal phalanx
                "distal_length": 0.025,    # 2.5cm distal phalanx
                "width": 0.018,            # 1.8cm width
                "base_pos": [-0.02, 0.035, 0],  # Position on palm
                "base_orientation": [0, 0, -40], # Degrees rotation
                "range_mcp": [-15, 90],     # MCP joint range (degrees)
                "range_ip": [0, 90],        # IP joint range (degrees)
                "relaxed_angles": [10, 10]  # Default angles in relaxed state
            },
            "Index": {
                "proximal_length": 0.04,   # 4cm proximal phalanx
                "distal_length": 0.025,    # 2.5cm distal phalanx
                "width": 0.016,            # 1.6cm width
                "base_pos": [0.03, 0.03, 0],  # Position on palm
                "base_orientation": [0, 0, 0], # Degrees rotation
                "range_mcp": [0, 90],       # MCP joint range (degrees)
                "range_pip": [0, 100],      # PIP joint range (degrees)
                "relaxed_angles": [0, 5]    # Default angles in relaxed state
            },
            "Middle": {
                "proximal_length": 0.042,  # 4.2cm proximal phalanx
                "distal_length": 0.028,    # 2.8cm distal phalanx
                "width": 0.016,            # 1.6cm width
                "base_pos": [0.03, 0.01, 0],  # Position on palm
                "base_orientation": [0, 0, 0], # Degrees rotation
                "range_mcp": [0, 90],       # MCP joint range (degrees)
                "range_pip": [0, 100],      # PIP joint range (degrees)
                "relaxed_angles": [0, 5]    # Default angles in relaxed state
            },
            "Ring": {
                "proximal_length": 0.039,  # 3.9cm proximal phalanx
                "distal_length": 0.025,    # 2.5cm distal phalanx
                "width": 0.015,            # 1.5cm width
                "base_pos": [0.03, -0.01, 0],  # Position on palm
                "base_orientation": [0, 0, 0], # Degrees rotation
                "range_mcp": [0, 90],       # MCP joint range (degrees)
                "range_pip": [0, 100],      # PIP joint range (degrees)
                "relaxed_angles": [0, 5]    # Default angles in relaxed state
            },
            "Pinky": {
                "proximal_length": 0.035,  # 3.5cm proximal phalanx
                "distal_length": 0.02,     # 2.0cm distal phalanx
                "width": 0.014,            # 1.4cm width
                "base_pos": [0.03, -0.03, 0],  # Position on palm
                "base_orientation": [0, 0, 0], # Degrees rotation
                "range_mcp": [0, 90],       # MCP joint range (degrees)
                "range_pip": [0, 100],      # PIP joint range (degrees)
                "relaxed_angles": [0, 5]    # Default angles in relaxed state
            }
        }
        
        # IMU positions on the hand model
        self.imu_positions = {
            "back": [0.0, 0.0, 0.02],     # Center of back of palm
            "wrist": [-0.04, 0.0, 0.01]   # Wrist position
        }
        
        # Sensor-to-joint-angle mapping
        # Maps proximity sensor distance to joint angle
        self.sensor_mapping = {
            "threshold_distance": 30,   # mm, distance at which movement starts
            "contact_distance": 5,      # mm, distance at which contact is detected
            "max_angle_mcp": 90,        # degrees, maximum MCP joint angle
            "max_angle_pip": 100        # degrees, maximum PIP/IP joint angle
        }
    
    def _add_meshes(self, root):
        """Add mesh assets to the model"""
        assets = ET.SubElement(root, "asset")
        
        # Add materials
        ET.SubElement(assets, "material", name="palm_material", rgba="0.5 0.5 0.5 1")
        ET.SubElement(assets, "material", name="finger_material", rgba="0.7 0.7 0.7 1")
        ET.SubElement(assets, "material", name="joint_material", rgba="0.3 0.3 0.3 1")
        ET.SubElement(assets, "material", name="imu_material", rgba="0.1 0.5 0.8 1")
    
    def _add_finger(self, parent, finger_name):
        """Add a finger with appropriate joints to the model"""
        params = self.finger_params[finger_name]
        
        # Create base of finger (attached to palm)
        base_pos = params["base_pos"]
        base_orientation = params["base_orientation"]
        
        # Base joint connecting to palm
        base_joint = ET.SubElement(parent, "body", name=f"{finger_name}_base", pos=f"{base_pos[0]} {base_pos[1]} {base_pos[2]}")
        ET.SubElement(base_joint, "site", name=f"{finger_name}_base_site", size="0.005", rgba="1 0 0 0.5")
        
        # MCP joint
        mcp_joint = ET.SubElement(base_joint, "body", name=f"{finger_name}_proximal", 
                                  pos="0 0 0", euler=f"{base_orientation[0]} {base_orientation[1]} {base_orientation[2]}")
        
        # Joint properties
        ET.SubElement(mcp_joint, "joint", name=f"{finger_name}_mcp", type="hinge", axis="0 1 0", 
                      range=f"{params['range_mcp'][0]} {params['range_mcp'][1]}")
        
        # Proximal phalanx geometry
        ET.SubElement(mcp_joint, "geom", name=f"{finger_name}_proximal_geom", type="capsule", 
                      size=f"{params['width']/2}", fromto=f"0 0 0 {params['proximal_length']} 0 0",
                      rgba="0.7 0.7 0.7 1", material="finger_material")
        
        # PIP/IP joint
        pip_joint = ET.SubElement(mcp_joint, "body", name=f"{finger_name}_distal", 
                                  pos=f"{params['proximal_length']} 0 0")
        
        # Joint properties
        if finger_name == "Thumb":
            joint_name = f"{finger_name}_ip"  # Interphalangeal joint for thumb
            range_key = "range_ip"
        else:
            joint_name = f"{finger_name}_pip"  # Proximal interphalangeal joint for fingers
            range_key = "range_pip"
            
        ET.SubElement(pip_joint, "joint", name=joint_name, type="hinge", axis="0 1 0", 
                      range=f"{params[range_key][0]} {params[range_key][1]}")
        
        # Distal phalanx geometry
        ET.SubElement(pip_joint, "geom", name=f"{finger_name}_distal_geom", type="capsule", 
                      size=f"{params['width']/2}", fromto=f"0 0 0 {params['distal_length']} 0 0",
                      rgba="0.7 0.7 0.7 1", material="finger_material")
        
        # Fingertip site (for visualization and contact detection)
        ET.SubElement(pip_joint, "site", name=f"{finger_name}_tip_site", 
                      pos=f"{params['distal_length']} 0 0", size="0.005", rgba="0 1 0 0.5")
    
    def _add_palm(self, parent):
        """Add the palm to the model"""
        palm = ET.SubElement(parent, "body", name="palm", pos="0 0 0")
        
        # Palm geometry
        ET.SubElement(palm, "geom", name="palm_geom", type="box", 
                      size=f"{self.palm_length/2} {self.palm_width/2} {self.palm_height/2}",
                      pos="0 0 0", rgba="0.5 0.5 0.5 1", material="palm_material")
        
        # Add IMU sensors as visual elements
        for imu_name, imu_pos in self.imu_positions.items():
            ET.SubElement(palm, "site", name=f"{imu_name}_imu_site", 
                          pos=f"{imu_pos[0]} {imu_pos[1]} {imu_pos[2]}", 
                          size="0.01", rgba="0.1 0.5 0.8 1", material="imu_material")
        
        # Add fingers to palm
        for finger_name in self.finger_params:
            self._add_finger(palm, finger_name)
        
        return palm
    
    def _add_actuators(self, root):
        """Add actuators for controlling joint movements"""
        actuators = ET.SubElement(root, "actuator")
        
        # Add position actuators for each finger joint
        for finger_name in self.finger_params:
            # MCP joint actuator
            ET.SubElement(actuators, "position", name=f"{finger_name}_mcp_actuator",
                          joint=f"{finger_name}_mcp", kp="10")
            
            # PIP/IP joint actuator
            if finger_name == "Thumb":
                joint_name = f"{finger_name}_ip"
            else:
                joint_name = f"{finger_name}_pip"
                
            ET.SubElement(actuators, "position", name=f"{finger_name}_pip_actuator",
                          joint=joint_name, kp="10")
    
    def _add_sensors(self, root):
        """Add sensors for the model"""
        sensors = ET.SubElement(root, "sensor")
        
        # Joint angle sensors
        for finger_name in self.finger_params:
            ET.SubElement(sensors, "jointpos", name=f"{finger_name}_mcp_sensor", joint=f"{finger_name}_mcp")
            
            if finger_name == "Thumb":
                joint_name = f"{finger_name}_ip"
            else:
                joint_name = f"{finger_name}_pip"
                
            ET.SubElement(sensors, "jointpos", name=f"{finger_name}_pip_sensor", joint=joint_name)
    
    def generate_base_model(self):
        """Generate the base XML model of the hand"""
        # Create root MuJoCo element
        root = ET.Element("mujoco", model="prosthetic_hand")
        
        # Add compiler options
        ET.SubElement(root, "compiler", angle="degree", coordinate="local", eulerseq="xyz")
        
        # Add options
        option = ET.SubElement(root, "option")
        option.set("timestep", "0.001")
        option.set("gravity", "0 0 0")  # No gravity as requested
        option.set("iterations", "100")
        option.set("integrator", "Euler")
        
        # Add asset meshes and materials
        self._add_meshes(root)
        
        # Add world body
        worldbody = ET.SubElement(root, "worldbody")
        
        # Add a global light
        ET.SubElement(worldbody, "light", directional="true", diffuse=".8 .8 .8", specular=".2 .2 .2", pos="0 0 1", dir="0 0 -1")
        
        # Add the palm with fingers
        self._add_palm(worldbody)
        
        # Add actuators
        self._add_actuators(root)
        
        # Add sensors
        self._add_sensors(root)
        
        # Format and save the XML
        xml_str = ET.tostring(root, encoding="unicode")
        pretty_xml = self._prettify_xml(xml_str)
        
        # Save the model
        model_path = self.output_dir / "hand_model.xml"
        with open(model_path, "w") as f:
            f.write(pretty_xml)
        
        print(f"Base hand model generated at: {model_path}")
        return model_path
    
    def _prettify_xml(self, xml_str):
        """Make the XML output more readable with proper indentation"""
        import xml.dom.minidom
        dom = xml.dom.minidom.parseString(xml_str)
        return dom.toprettyxml(indent="  ")
    
    def create_static_pose_model(self, pose_name="relaxed"):
        """Create a model with a specific static pose"""
        # Generate the base model first
        base_model_path = self.generate_base_model()
        
        # Parse the base model
        tree = ET.parse(base_model_path)
        root = tree.getroot()
        
        # Find the keyframe element or create it
        keyframes = root.find("keyframe")
        if keyframes is None:
            keyframes = ET.SubElement(root, "keyframe")
        
        # Create a keyframe for the specified pose
        if pose_name == "relaxed":
            # Create relaxed pose keyframe
            keyframe = ET.SubElement(keyframes, "key", name="relaxed_pose")
            
            # Set each joint to its relaxed angle
            for finger_name, params in self.finger_params.items():
                # MCP joint
                joint_id = f"{finger_name}_mcp"
                mcp_value = str(params["relaxed_angles"][0])
                ET.SubElement(keyframe, "joint", joint=joint_id, value=mcp_value)
                
                # PIP/IP joint
                if finger_name == "Thumb":
                    joint_id = f"{finger_name}_ip"
                else:
                    joint_id = f"{finger_name}_pip"
                pip_value = str(params["relaxed_angles"][1])
                ET.SubElement(keyframe, "joint", joint=joint_id, value=pip_value)
        
        # Save the model with keyframe
        pose_model_path = self.output_dir / f"hand_model_{pose_name}.xml"
        tree.write(pose_model_path, encoding="unicode", xml_declaration=True)
        
        print(f"Static pose model '{pose_name}' generated at: {pose_model_path}")
        return pose_model_path
    
    def convert_sensor_to_angle(self, distance, is_mcp=True, finger="Index"):
        """Convert proximity sensor distance to joint angle
        
        Args:
            distance: Proximity sensor distance in mm
            is_mcp: Whether this is for MCP joint (True) or PIP/IP (False)
            finger: Finger name
            
        Returns:
            Joint angle in degrees
        """
        # Get parameters
        threshold = self.sensor_mapping["threshold_distance"]
        contact = self.sensor_mapping["contact_distance"]
        
        # Determine max angle based on joint type and finger
        if is_mcp:
            max_angle = self.sensor_mapping["max_angle_mcp"]
        else:
            max_angle = self.sensor_mapping["max_angle_pip"]
        
        # If beyond threshold, no bending
        if distance >= threshold:
            return 0.0
            
        # If at or below contact, full bending
        if distance <= contact:
            return max_angle
            
        # Otherwise, proportional mapping
        normalized = (threshold - distance) / (threshold - contact)
        angle = normalized * max_angle
        return angle
    
    def create_data_replay_model(self, data_file, output_name="replay_model"):
        """Create a MuJoCo model with keyframes from recorded data
        
        Args:
            data_file: Path to the recorded data JSON file
            output_name: Name for the output model file
            
        Returns:
            Path to the generated model file
        """
        import json
        import pandas as pd
        
        # Load the data file
        data_path = Path(data_file)
        
        try:
            # Try to load as JSON first
            with open(data_path, 'r') as f:
                data = json.load(f)
            df = pd.DataFrame(data)
        except:
            # If not JSON, try loading as CSV
            try:
                df = pd.read_csv(data_path)
            except Exception as e:
                print(f"Error loading data file: {e}")
                return None
        
        # Generate the base model
        base_model_path = self.generate_base_model()
        
        # Parse the base model
        tree = ET.parse(base_model_path)
        root = tree.getroot()
        
        # Find the keyframe element or create it
        keyframes = root.find("keyframe")
        if keyframes is None:
            keyframes = ET.SubElement(root, "keyframe")
        
        # Create keyframes from the data (sample every N rows to avoid too many keyframes)
        sample_interval = max(1, len(df) // 100)  # Sample to get about 100 keyframes
        
        for idx, row in df.iloc[::sample_interval].iterrows():
            # Create a keyframe for this time step
            frame_time = row.get("time_seconds", idx * 0.1)  # Use time_seconds if available
            keyframe = ET.SubElement(keyframes, "key", name=f"frame_{idx}", time=f"{frame_time}")
            
            # Process each finger
            for finger_name in self.finger_params.keys():
                # Get sensor values for this finger
                sensor_name = f"{finger_name}1"  # MCP joint sensor
                
                # Get the filtered distance value if available
                if f"{sensor_name}_filtered" in row:
                    distance = row[f"{sensor_name}_filtered"]
                    
                    # Convert sensor readings to joint angles
                    mcp_angle = self.convert_sensor_to_angle(distance, is_mcp=True, finger=finger_name)
                    pip_angle = self.convert_sensor_to_angle(distance, is_mcp=False, finger=finger_name)
                    
                    # Add MCP joint keyframe
                    ET.SubElement(keyframe, "joint", joint=f"{finger_name}_mcp", value=str(mcp_angle))
                    
                    # Add PIP/IP joint keyframe
                    if finger_name == "Thumb":
                        joint_id = f"{finger_name}_ip"
                    else:
                        joint_id = f"{finger_name}_pip"
                    ET.SubElement(keyframe, "joint", joint=joint_id, value=str(pip_angle))
            
            # Add IMU orientation if available
            if "back_imu_roll" in row and "back_imu_pitch" in row and "back_imu_yaw" in row:
                palm_body = root.find(f".//body[@name='palm']")
                if palm_body is not None:
                    # MuJoCo uses degrees for joint angles
                    palm_body.set("euler", f"{row['back_imu_roll']} {row['back_imu_pitch']} {row['back_imu_yaw']}")
        
        # Save the model with keyframes
        replay_model_path = self.output_dir / f"{output_name}.xml"
        tree.write(replay_model_path, encoding="unicode", xml_declaration=True)
        
        print(f"Data replay model generated at: {replay_model_path}")
        return replay_model_path
    
    def visualize_hand_dimensions(self):
        """Create a visualization of the hand dimensions for reference"""
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Draw palm
        palm_rect = plt.Rectangle((-self.palm_length/2, -self.palm_width/2), 
                                 self.palm_length, self.palm_width, 
                                 facecolor='lightgray', edgecolor='black', alpha=0.7)
        ax.add_patch(palm_rect)
        
        # Draw fingers
        for finger_name, params in self.finger_params.items():
            # Get base position
            base_pos = params["base_pos"]
            base_angle = np.radians(params["base_orientation"][2])
            
            # Calculate positions
            x0, y0 = base_pos[0], base_pos[1]
            prox_length = params["proximal_length"]
            dist_length = params["distal_length"]
            width = params["width"]
            
            # Draw proximal phalanx
            prox_angle = base_angle + np.radians(params["relaxed_angles"][0])
            x1 = x0 + prox_length * np.cos(prox_angle)
            y1 = y0 + prox_length * np.sin(prox_angle)
            
            plt.plot([x0, x1], [y0, y1], 'k-', linewidth=width*100)
            
            # Draw distal phalanx
            dist_angle = prox_angle + np.radians(params["relaxed_angles"][1])
            x2 = x1 + dist_length * np.cos(dist_angle)
            y2 = y1 + dist_length * np.sin(dist_angle)
            
            plt.plot([x1, x2], [y1, y2], 'k-', linewidth=width*100)
            
            # Label the finger
            plt.text(x2, y2, finger_name, fontsize=12)
        
        # Draw IMU positions
        for imu_name, imu_pos in self.imu_positions.items():
            plt.plot(imu_pos[0], imu_pos[1], 'bo', markersize=10)
            plt.text(imu_pos[0], imu_pos[1], f"{imu_name} IMU", fontsize=10)
        
        # Set plot properties
        ax.set_xlim(-0.1, 0.1)
        ax.set_ylim(-0.1, 0.1)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title('Hand Dimensions (Top View)')
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        
        # Save the visualization
        viz_path = self.output_dir / "hand_dimensions.png"
        plt.savefig(viz_path, dpi=300)
        
        print(f"Hand dimension visualization saved to: {viz_path}")
        plt.close()
        
        return viz_path


class MujocoSimulator:
    """Handles MuJoCo simulation and visualization of hand models"""
    
    def __init__(self, model_path=None):
        """Initialize the MuJoCo simulator
        
        Args:
            model_path: Path to the MuJoCo XML model file
        """
        self.model_path = model_path
        
        # Display instructions for running the simulation
        print("MuJoCo Simulator Setup Instructions:")
        print("1. Install MuJoCo 2.0+ and mujoco-py if not already installed:")
        print("   pip install mujoco-py")
        print()
        print("2. To visualize a static pose:")
        print("   python -c \"import mujoco_py; viewer = mujoco_py.MjViewer(mujoco_py.load_model_from_path('path/to/hand_model_relaxed.xml')); viewer.render()\"")
        print()
        print("3. To replay a recorded data session:")
        print("   python -c \"import mujoco_py; viewer = mujoco_py.MjViewer(mujoco_py.load_model_from_path('path/to/replay_model.xml')); viewer.render()\"")
        print()
        print("Use the following keyboard shortcuts in the MuJoCo viewer:")
        print("- Space: Play/pause animation")
        print("- Right arrow: Step forward one frame")
        print("- Left arrow: Step backward one frame")
        print("- 'r': Reset to initial state")
        
    def run_simulation(self):
        """Run the MuJoCo simulation interactively"""
        print("This function requires an interactive Python session.")
        print("Please use the command line instructions provided above to run the simulation.")
        print("Alternatively, implement this function with your preferred MuJoCo Python binding.")


def generate_default_models():
    """Generate default hand models for visualization and replay"""
    # Create the model generator
    generator = HandModelGenerator(output_dir="mujoco_models")
    
    # Generate the base model
    base_model_path = generator.generate_base_model()
    
    # Generate a relaxed pose model
    relaxed_model_path = generator.create_static_pose_model(pose_name="relaxed")
    
    # Visualize the hand dimensions
    viz_path = generator.visualize_hand_dimensions()
    
    # Set up the simulator with instructions
    simulator = MujocoSimulator(model_path=relaxed_model_path)
    
    print(f"\nModels generated successfully:")
    print(f"- Base model: {base_model_path}")
    print(f"- Relaxed pose model: {relaxed_model_path}")
    print(f"- Hand visualization: {viz_path}")
    print("\nUse the MujocoSimulator instructions to visualize the models.")

if __name__ == "__main__":
    generate_default_models()