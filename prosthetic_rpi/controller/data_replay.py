# Data Replay: Replays recorded data sessions and reconstructs control states

import os
import json
import csv
import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path

class DataReplay:
    """Loads and replays recorded data sessions
    
    This module allows for:
    1. Loading and parsing saved data sessions
    2. Recreating control states and decisions
    3. Visualizing recorded data
    4. Analyzing performance and decision boundaries
    """
    
    def __init__(self, session_path=None):
        """Initialize the data replay system
        
        Args:
            session_path: Path to a recorded session directory
        """
        self.session_path = Path(session_path) if session_path else None
        self.data = None
        self.metadata = None
        
        # Load data if a session path was provided
        if self.session_path:
            self.load_session(self.session_path)
    
    def load_session(self, session_path):
        """Load a recorded data session
        
        Args:
            session_path: Path to the session directory
            
        Returns:
            True if session loaded successfully, False otherwise
        """
        self.session_path = Path(session_path)
        
        # Check if directory exists
        if not self.session_path.exists() or not self.session_path.is_dir():
            print(f"Error: Session directory {session_path} not found")
            return False
        
        # Try to load metadata
        try:
            with open(self.session_path / "metadata.json", 'r') as f:
                self.metadata = json.load(f)
        except FileNotFoundError:
            print("Error: Metadata file not found in session directory")
            return False
        
        # Try to load complete dataset from JSON first (it's easier to work with)
        try:
            with open(self.session_path / "complete_dataset.json", 'r') as f:
                self.data = json.load(f)
            print(f"Loaded {len(self.data)} records from JSON dataset")
            return True
        except FileNotFoundError:
            pass  # Fall back to CSV
        
        # Fall back to loading from CSV if JSON not available
        try:
            self.data = []
            with open(self.session_path / "sensor_data.csv", 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Convert numeric values from strings
                    for key, value in row.items():
                        if key == 'timestamp':
                            row[key] = float(value)
                        elif key.endswith('_raw') or key.endswith('_filtered') or key.endswith('_current'):
                            row[key] = float(value)
                    self.data.append(row)
            print(f"Loaded {len(self.data)} records from CSV dataset")
            return True
        except FileNotFoundError:
            print("Error: Neither complete_dataset.json nor sensor_data.csv found")
            return False
    
    def get_available_sessions(self, base_dir="/home/pi/hand_data"):
        """Find available data sessions
        
        Args:
            base_dir: Base directory to search for sessions
            
        Returns:
            List of session directories
        """
        base_path = Path(base_dir)
        if not base_path.exists():
            return []
        
        # Find directories that start with "session_"
        sessions = [d for d in base_path.iterdir() 
                   if d.is_dir() and d.name.startswith("session_")]
        
        # Sort by name (which includes timestamp)
        sessions.sort()
        
        return sessions
    
    def convert_to_dataframe(self):
        """Convert the loaded data to a pandas DataFrame for easier analysis
        
        Returns:
            Pandas DataFrame containing session data
        """
        if self.data is None:
            print("No data loaded")
            return None
        
        # Convert to DataFrame
        df = pd.DataFrame(self.data)
        
        # Convert timestamp to datetime
        if 'timestamp' in df.columns:
            df['time_seconds'] = df['timestamp'] - df['timestamp'].iloc[0]
        
        return df
    
    def reconstruct_control_states(self, approach_threshold=40, contact_threshold=5):
        """Reconstruct control states from sensor data
        
        This verifies that the saved control mode matches what would be decided
        based on the recorded sensor values, validating the control algorithm.
        
        Args:
            approach_threshold: Threshold for approach control mode (mm)
            contact_threshold: Threshold for contact control mode (mm)
            
        Returns:
            DataFrame with reconstructed control states
        """
        df = self.convert_to_dataframe()
        if df is None:
            return None
        
        # List of fingers to process
        fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        
        # Function to determine control mode from distance
        def determine_mode(distance):
            if distance >= approach_threshold:
                return "APPROACH"
            elif distance <= contact_threshold:
                return "CONTACT"
            else:
                return "PROPORTIONAL"
        
        # Reconstruct modes for each finger
        for finger in fingers:
            sensor_name = f"{finger}1"  # e.g., "Thumb1" for "Thumb"
            if f"{sensor_name}_filtered" in df.columns:
                # Create reconstructed mode column
                df[f"{finger}_reconstructed_mode"] = df[f"{sensor_name}_filtered"].apply(determine_mode)
                
                # Check if modes match (if mode column exists)
                if f"{finger}_mode" in df.columns:
                    df[f"{finger}_mode_match"] = df[f"{finger}_mode"] == df[f"{finger}_reconstructed_mode"]
        
        return df
    
    def plot_finger_data(self, finger_name, start_time=None, end_time=None):
        """Plot data for a specific finger
        
        Args:
            finger_name: Name of the finger (e.g., "Thumb")
            start_time: Start time for plot in seconds (optional)
            end_time: End time for plot in seconds (optional)
            
        Returns:
            Matplotlib figure
        """
        df = self.convert_to_dataframe()
        if df is None:
            return None
        
        # Filter by time range if specified
        if start_time is not None or end_time is not None:
            if start_time is not None:
                df = df[df['time_seconds'] >= start_time]
            if end_time is not None:
                df = df[df['time_seconds'] <= end_time]
        
        # Get sensor name
        sensor_name = f"{finger_name}1"  # e.g., "Thumb1" for "Thumb"
        
        # Create figure with multiple subplots
        fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        
        # Plot sensor readings
        if f"{sensor_name}_raw" in df.columns and f"{sensor_name}_filtered" in df.columns:
            axs[0].plot(df['time_seconds'], df[f"{sensor_name}_raw"], 
                        label=f"{sensor_name} Raw", color='lightblue', alpha=0.6)
            axs[0].plot(df['time_seconds'], df[f"{sensor_name}_filtered"], 
                        label=f"{sensor_name} Filtered", color='blue')
            axs[0].set_ylabel("Distance (mm)")
            axs[0].set_title(f"{finger_name} Proximity Sensor Readings")
            axs[0].legend()
            axs[0].grid(True)
            
            # Add threshold lines
            axs[0].axhline(y=40, color='green', linestyle='--', alpha=0.7, label="Approach threshold")
            axs[0].axhline(y=5, color='red', linestyle='--', alpha=0.7, label="Contact threshold")
        
        # Plot motor current
        if f"{finger_name}_current" in df.columns:
            axs[1].plot(df['time_seconds'], df[f"{finger_name}_current"], 
                        label=f"{finger_name} Current", color='orange')
            axs[1].set_ylabel("Current (A)")
            axs[1].set_title(f"{finger_name} Motor Current")
            axs[1].legend()
            axs[1].grid(True)
        
        # Plot control mode
        if f"{finger_name}_mode" in df.columns:
            # Convert mode strings to numeric for plotting
            mode_map = {"IDLE": 0, "APPROACH": 1, "PROPORTIONAL": 2, "CONTACT": 3}
            df['mode_numeric'] = df[f"{finger_name}_mode"].map(mode_map)
            
            axs[2].step(df['time_seconds'], df['mode_numeric'], 
                        label=f"{finger_name} Mode", color='purple', where='post')
            axs[2].set_yticks([0, 1, 2, 3])
            axs[2].set_yticklabels(["IDLE", "APPROACH", "PROPORTIONAL", "CONTACT"])
            axs[2].set_ylabel("Control Mode")
            axs[2].set_title(f"{finger_name} Control Mode")
            axs[2].grid(True)
        
        # Set common x-axis label
        axs[2].set_xlabel("Time (seconds)")
        
        plt.tight_layout()
        return fig
    
    def analyze_phase_transitions(self):
        """Analyze control phase transitions to measure system responsiveness
        
        Returns:
            DataFrame with phase transition analysis
        """
        df = self.convert_to_dataframe()
        if df is None:
            return None
        
        # List of fingers to process
        fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        
        results = []
        
        for finger in fingers:
            # Check if mode column exists
            if f"{finger}_mode" not in df.columns:
                continue
                
            # Find all transitions
            transitions = []
            prev_mode = None
            
            for i, row in df.iterrows():
                current_mode = row[f"{finger}_mode"]
                
                if prev_mode is not None and current_mode != prev_mode:
                    # Found a transition
                    transitions.append({
                        "finger": finger,
                        "from_mode": prev_mode,
                        "to_mode": current_mode,
                        "time": row["time_seconds"],
                        "distance": row[f"{finger}1_filtered"]
                    })
                
                prev_mode = current_mode
            
            results.extend(transitions)
        
        return pd.DataFrame(results)
    
    def replay_session(self, replay_factor=1.0):
        """Replay the session in real-time or scaled time
        
        Args:
            replay_factor: Speed factor (1.0 = real-time, 2.0 = 2x speed)
        """
        if self.data is None or len(self.data) == 0:
            print("No data to replay")
            return
        
        df = self.convert_to_dataframe()
        if 'time_seconds' not in df.columns:
            print("Cannot replay: missing timestamp information")
            return
        
        # Calculate time between records
        time_diffs = np.diff(df['time_seconds'].values)
        
        print(f"Starting replay (factor: {replay_factor}x)")
        print("Press Ctrl+C to stop")
        
        try:
            for i in range(len(df) - 1):
                # Print current state
                record = df.iloc[i].to_dict()
                
                # Get finger information for display
                fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
                
                # Print header every 20 records
                if i % 20 == 0:
                    print("\n" + "-" * 80)
                    print(f"{'Time':>8} | {'Finger':>8} | {'Distance':>8} | {'Mode':>12} | {'Current':>8}")
                    print("-" * 80)
                
                # Print data for each finger
                for finger in fingers:
                    if f"{finger}_mode" in record:
                        distance = record.get(f"{finger}1_filtered", "N/A")
                        mode = record.get(f"{finger}_mode", "N/A")
                        current = record.get(f"{finger}_current", "N/A")
                        
                        if isinstance(distance, (int, float)) and isinstance(current, (int, float)):
                            print(f"{record['time_seconds']:8.2f} | {finger:>8} | {distance:8.1f} | {mode:>12} | {current:8.3f}")
                
                # Wait appropriate time for next record
                wait_time = time_diffs[i] / replay_factor
                if wait_time > 0:
                    time.sleep(wait_time)
                    
        except KeyboardInterrupt:
            print("\nReplay stopped by user")
        
        print("Replay complete")