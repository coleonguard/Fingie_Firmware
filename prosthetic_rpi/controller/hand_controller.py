# Hand Controller: Main control algorithm coordinating sensors and actuators

import time
import threading
from enum import Enum

# Import our custom modules
from .sensors import ProximitySensorManager, MCP_SENSORS
from .actuators import MotorController
from .data_logger import DataLogger
from .imu_sensor import IMUManager

class ControlMode(Enum):
    """Control modes for the prosthetic hand"""
    IDLE = 0        # No control, motors off
    APPROACH = 1    # Beyond threshold, no movement
    PROPORTIONAL = 2  # Between threshold and contact, proportional control
    CONTACT = 3     # Object contact detected, torque control

class HandController:
    """Main controller for the prosthetic hand
    
    Implements the theory of operation:
    1. Read proximity sensors at MCP joints
    2. Apply Kalman filtering
    3. Determine control mode based on distance thresholds
    4. Apply appropriate control strategy:
       - Beyond threshold: No movement
       - Between threshold and contact: Proportional position control
       - Contact detected: Switch to torque control
    """
    
    def __init__(self, update_rate=20, enable_logging=True, enable_imu=True, log_path="/home/pi/hand_data"):  # 20Hz default control rate
        """Initialize the hand controller
        
        Args:
            update_rate: Frequency to update control decisions (Hz)
            enable_logging: Whether to enable data logging
            enable_imu: Whether to enable IMU orientation tracking
            log_path: Base directory for log files
        """
        # Initialize sensors and actuators
        self.sensors = ProximitySensorManager(sampling_rate=update_rate*2)  # Sample at 2x control rate
        self.motors = MotorController()
        
        # Initialize IMU manager
        self.enable_imu = enable_imu
        self.imu_manager = None
        if self.enable_imu:
            self.imu_manager = IMUManager(sampling_rate=update_rate)
            
        # Initialize data logger
        self.enable_logging = enable_logging
        self.log_path = log_path
        self.data_logger = None
        if self.enable_logging:
            self.data_logger = DataLogger(base_dir=self.log_path, log_rate=update_rate)
        
        # Control parameters
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate
        
        # Control thresholds (in mm)
        self.approach_threshold = 40  # Distance beyond which no action occurs
        self.contact_threshold = 5    # Distance at which contact is detected
        
        # Current control parameters
        self.max_current = 0.8        # Maximum current during proportional control (A)
        self.contact_current = 0.5    # Current during contact for stable grip (A)
        
        # Finger names mapping
        self.finger_mapping = {
            "Thumb1": "Thumb",
            "Index1": "Index",
            "Middle1": "Middle",
            "Ring1": "Ring",
            "Pinky1": "Pinky"
        }
        
        # Current control mode for each finger
        self.control_modes = {finger: ControlMode.IDLE for finger in self.finger_mapping.values()}
        
        # Threading control
        self.running = False
        self.thread = None
    
    def _determine_control_mode(self, distance):
        """Determine control mode based on current distance
        
        Args:
            distance: Current distance reading (mm)
            
        Returns:
            Appropriate ControlMode enum value
        """
        if distance >= self.approach_threshold:
            return ControlMode.APPROACH
        elif distance <= self.contact_threshold:
            return ControlMode.CONTACT
        else:
            return ControlMode.PROPORTIONAL
    
    def _calculate_proportional_current(self, distance):
        """Calculate current for proportional control based on distance
        
        Args:
            distance: Current distance reading (mm)
            
        Returns:
            Appropriate current value (A)
        """
        # Linear mapping from distance to current
        # At approach_threshold → 0 current
        # At contact_threshold → max_current
        
        # Normalize the distance to [0, 1] range
        range_size = self.approach_threshold - self.contact_threshold
        if range_size <= 0:  # Safety check
            return 0.0
            
        normalized_distance = (self.approach_threshold - distance) / range_size
        normalized_distance = max(0.0, min(1.0, normalized_distance))  # Clamp to [0, 1]
        
        # Apply to current range
        current = normalized_distance * self.max_current
        return current
    
    def _control_loop(self):
        """Main control loop for hand controller"""
        next_update_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next control update
            if current_time >= next_update_time:
                # Process all fingers
                self._update_all_fingers()
                
                # Calculate next update time
                next_update_time = current_time + self.update_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def _update_all_fingers(self):
        """Process control for all fingers"""
        # For each MCP sensor, determine control mode and set appropriate current
        for sensor_name in MCP_SENSORS:
            # Get the corresponding finger name
            finger_name = self.finger_mapping.get(sensor_name)
            if not finger_name:
                continue
                
            # Get raw and filtered sensor readings
            raw_distance = self.sensors.get_sensor_value(sensor_name, filtered=False)
            filtered_distance = self.sensors.get_sensor_value(sensor_name, filtered=True)
            
            # Determine control mode
            control_mode = self._determine_control_mode(filtered_distance)
            self.control_modes[finger_name] = control_mode
            
            # Calculate current based on control mode
            current = 0.0  # Default to no current
            
            if control_mode == ControlMode.APPROACH:
                # No movement in approach mode
                current = 0.0
                
            elif control_mode == ControlMode.PROPORTIONAL:
                # Proportional control based on distance
                current = self._calculate_proportional_current(filtered_distance)
                
            elif control_mode == ControlMode.CONTACT:
                # Contact detected, switch to torque control
                current = self.contact_current
            
            # Set the current on the motor
            self.motors.set_motor_current(finger_name, current)
            
            # Log data if enabled
            if self.data_logger:
                # Log sensor data
                self.data_logger.update_sensor_data(sensor_name, raw_distance, filtered_distance)
                
                # Log motor and control data
                self.data_logger.update_motor_data(finger_name, self.motors.get_actual_current(finger_name))
                self.data_logger.update_control_mode(finger_name, control_mode.name)
        
        # Log IMU data if enabled and available
        if self.data_logger and self.imu_manager:
            # Get back of hand IMU data
            back_data = self.imu_manager.get_orientation("back")
            self.data_logger.update_imu_data("back", back_data.roll, back_data.pitch, back_data.yaw)
            
            # Get wrist IMU data
            wrist_data = self.imu_manager.get_orientation("wrist")
            self.data_logger.update_imu_data("wrist", wrist_data.roll, wrist_data.pitch, wrist_data.yaw)
    
    def start(self):
        """Start the hand controller system"""
        if not self.running:
            # Start the sensor and motor subsystems
            self.sensors.start()
            self.motors.start()
            
            # Start IMU manager if enabled
            if self.imu_manager:
                self.imu_manager.start()
            
            # Start data logger if enabled
            if self.data_logger:
                self.data_logger.start()
            
            # Start the main control thread
            self.running = True
            self.thread = threading.Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            
            print("Hand controller started")
    
    def stop(self):
        """Stop the hand controller system"""
        # Stop the main control thread
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Stop subsystems
        self.motors.stop()
        self.sensors.stop()
        
        # Stop IMU manager if enabled
        if self.imu_manager:
            self.imu_manager.stop()
        
        # Stop data logger if enabled
        if self.data_logger:
            self.data_logger.stop()
        
        print("Hand controller stopped")
        
    def save_snapshot(self, description="manual_snapshot"):
        """Save a snapshot of the current state with a description
        
        Args:
            description: Text description for the snapshot
        """
        if self.data_logger:
            self.data_logger.save_snapshot(description)
            return True
        else:
            print("Data logging not enabled, cannot save snapshot")
            return False
    
    def get_finger_info(self, finger_name):
        """Get current information about a specific finger
        
        Args:
            finger_name: Name of the finger ("Thumb", "Index", etc.)
            
        Returns:
            Dictionary with finger information
        """
        # Find the sensor name for this finger
        sensor_name = None
        for s, f in self.finger_mapping.items():
            if f == finger_name:
                sensor_name = s
                break
                
        if not sensor_name:
            return {"error": "Invalid finger name"}
            
        # Get current state information
        distance = self.sensors.get_sensor_value(sensor_name, filtered=True)
        mode = self.control_modes.get(finger_name, ControlMode.IDLE)
        current = self.motors.get_actual_current(finger_name)
        
        return {
            "finger": finger_name,
            "distance": distance,
            "mode": mode.name,
            "current": current
        }
