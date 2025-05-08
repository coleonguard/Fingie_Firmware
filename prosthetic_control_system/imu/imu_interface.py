#!/usr/bin/env python3
"""
IMU Interface for Prosthetic Control.

This module implements the IMU interface for the Microstrain IMUs used 
in the prosthetic control system. It handles both orientation and 
acceleration data needed for table contact detection.
"""

import os
import time
import threading
import logging
import math
import numpy as np
from enum import Enum
from dataclasses import dataclass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("IMUInterface")

# Constants
IMU_RATE = 200  # Hz
GRAVITY = 9.81  # m/s²
ANGULAR_RATE_STATIONARY_THRESHOLD = 10.0  # degrees/s
ACCEL_STATIONARY_THRESHOLD = 1.0  # m/s²
IMPACT_THRESHOLD = 7.8  # m/s² (high-pass filtered)
LOWERING_THRESHOLD = GRAVITY - 4.0  # m/s²
LOWERING_MIN_DURATION = 0.05  # seconds (50ms)
STATIONARY_WINDOW = 0.3  # seconds

try:
    import mscl
    MSCL_AVAILABLE = True
except ImportError:
    logger.warning("MSCL library not available. Using simulated IMU.")
    MSCL_AVAILABLE = False

class MotionState(Enum):
    """States for the motion state machine"""
    UNKNOWN = 0
    STATIC = 1
    REACHING = 2
    LOWERING = 3
    IMPACT = 4
    STATIONARY = 5

@dataclass
class IMUData:
    """Container for IMU data"""
    # Orientation (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Linear acceleration (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    
    # Angular rates (degrees/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # Derived values
    accel_magnitude: float = 0.0
    gyro_magnitude: float = 0.0
    
    # High-pass filtered acceleration for impact detection
    accel_z_highpass: float = 0.0
    
    # Timestamp
    timestamp: float = 0.0

class OrientationError(Exception):
    """Exception raised when IMU orientation is incorrect"""
    pass

class IMUInterface:
    """
    Interface for Microstrain IMUs.
    
    This class provides access to IMU data and implements algorithms for
    detecting hand motion states (lowering, impact, stationary) needed for
    the hand controller.
    """
    
    def __init__(
        self,
        sampling_rate=200,
        primary_port="/dev/ttyACM0",
        secondary_port="/dev/ttyACM1",
        highpass_alpha=0.8,  # High-pass filter coefficient
        check_orientation=True,
    ):
        """
        Initialize the IMU interface.
        
        Args:
            sampling_rate: IMU data collection rate (Hz)
            primary_port: Serial port for primary IMU
            secondary_port: Serial port for secondary IMU
            highpass_alpha: High-pass filter coefficient for impact detection
            check_orientation: Whether to check IMU orientation on startup
        """
        self.sampling_rate = sampling_rate
        self.sampling_interval = 1.0 / sampling_rate
        self.primary_port = primary_port
        self.secondary_port = secondary_port
        self.highpass_alpha = highpass_alpha
        self.check_orientation = check_orientation
        
        # IMU objects
        self.primary_imu = None
        self.secondary_imu = None
        
        # Latest data
        self.current_data = IMUData()
        self.previous_data = IMUData()
        self.previous_raw_accel_z = 0.0  # For high-pass filter
        
        # Motion state
        self.motion_state = MotionState.UNKNOWN
        self.state_entry_time = 0.0
        self.lowering_start_time = 0.0
        self.impact_detection_time = 0.0
        self.stationary_history = []  # List of tuples (accel_z, gyro_mag)
        
        # Thread control
        self.running = False
        self.thread = None
        
        # Initialize IMUs if MSCL is available
        if MSCL_AVAILABLE:
            self._initialize_imus()
        else:
            logger.warning("Using simulated IMU - MSCL not available")
            self._setup_simulated_imu()
    
    def _enable_port(self, port):
        """Enable access to the IMU serial port"""
        try:
            os.system(f'sudo chmod 777 {port}')
        except Exception as e:
            logger.error(f"Failed to enable port {port}: {e}")
    
    def _initialize_imus(self):
        """Initialize the IMUs"""
        if not MSCL_AVAILABLE:
            logger.error("Cannot initialize IMUs - MSCL not available")
            return False
            
        # Primary IMU (back of hand)
        try:
            logger.info(f"Initializing primary IMU on {self.primary_port}")
            self._enable_port(self.primary_port)
            self.primary_imu = self._setup_imu(self.primary_port)
            logger.info("Primary IMU initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize primary IMU: {e}")
            
        # Secondary IMU (wrist)
        try:
            logger.info(f"Initializing secondary IMU on {self.secondary_port}")
            self._enable_port(self.secondary_port)
            self.secondary_imu = self._setup_imu(self.secondary_port)
            logger.info("Secondary IMU initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize secondary IMU: {e}")
        
        # Verify at least one IMU is available
        if self.primary_imu is None and self.secondary_imu is None:
            logger.error("Failed to initialize any IMU")
            return False
        
        # Check orientation if requested
        if self.check_orientation and self.primary_imu is not None:
            self._check_orientation()
        
        return True
    
    def _check_orientation(self):
        """
        Check that the IMU's Z-axis is oriented correctly.
        
        In the expected orientation, Z acceleration should be close to -g.
        
        Raises:
            OrientationError: If the IMU has incorrect orientation
        """
        try:
            data = self._get_imu_data(self.primary_imu)
            # In normal orientation, Z acceleration should be close to -g
            if data.accel_z > -5.0:  # Not strongly negative
                raise OrientationError(
                    f"IMU Z-axis orientation incorrect: got {data.accel_z:.2f} m/s², "
                    f"expected approximately {-GRAVITY:.2f} m/s²"
                )
            logger.info("IMU orientation verified")
        except OrientationError:
            # Re-raise OrientationError
            raise
        except Exception as e:
            logger.error(f"IMU orientation check failed: {e}")
            # Re-raise as OrientationError
            raise OrientationError(f"IMU orientation check failed: {e}")
        
        return True
    
    def _setup_imu(self, port):
        """Set up a single IMU with the specified configuration"""
        try:
            # Create a connection
            connection = mscl.Connection.Serial(port)
            imu_node = mscl.InertialNode(connection)
            
            # Configure channels (Euler angles + accel + gyro)
            channels = mscl.MipChannels()
            
            # Add Euler angles
            channels.append(
                mscl.MipChannel(
                    mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES,
                    mscl.SampleRate.Hertz(self.sampling_rate)
                )
            )
            
            # Add scaled acceleration
            channels.append(
                mscl.MipChannel(
                    mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
                    mscl.SampleRate.Hertz(self.sampling_rate)
                )
            )
            
            # Add gyro
            channels.append(
                mscl.MipChannel(
                    mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC,
                    mscl.SampleRate.Hertz(self.sampling_rate)
                )
            )
            
            # Apply configuration
            imu_node.setToIdle()
            imu_node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, channels)
            imu_node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)
            imu_node.resume()
            
            return imu_node
            
        except Exception as e:
            logger.error(f"Failed to set up IMU on {port}: {e}")
            raise
    
    def _setup_simulated_imu(self):
        """Set up a simulated IMU for testing"""
        class SimulatedIMU:
            def __init__(self):
                self.time = 0.0
                
            def getDataPackets(self, timeout):
                """Return simulated data packets"""
                self.time += 0.01
                
                # Simulate sinusoidal motion
                class DataPoint:
                    def __init__(self, field_value, qualifier_value, data_value):
                        self._field = field_value
                        self._qualifier = qualifier_value
                        self._value = data_value
                    
                    def field(self):
                        return self._field
                        
                    def qualifier(self):
                        return self._qualifier
                        
                    def as_double(self):
                        return self._value
                
                class DataPacket:
                    def __init__(self, points):
                        self._data_points = points
                        
                    def data(self):
                        return self._data_points
                
                # Simulate sinusoidal movement
                t = self.time
                roll = 5 * math.sin(t * 0.5)
                pitch = 8 * math.sin(t * 0.7)
                yaw = 10 * math.sin(t * 0.3)
                
                # Simulate acceleration - gravity plus some noise
                accel_x = 0.5 * math.sin(t * 2.0)
                accel_y = 0.8 * math.sin(t * 2.2)
                accel_z = -GRAVITY + 0.3 * math.sin(t * 2.5)
                
                # Simulate gyro
                gyro_x = 5 * math.cos(t * 1.5)
                gyro_y = 8 * math.cos(t * 1.8)
                gyro_z = 3 * math.cos(t * 1.2)
                
                # Create data points
                data_points = [
                    # Euler angles
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES, 6, roll),
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES, 7, pitch),
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES, 8, yaw),
                    
                    # Acceleration
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC, 1, accel_x),
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC, 2, accel_y),
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC, 3, accel_z),
                    
                    # Gyro
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC, 1, gyro_x),
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC, 2, gyro_y),
                    DataPoint(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC, 3, gyro_z),
                ]
                
                # Return in a packet
                packet = DataPacket(data_points)
                return [packet]
        
        # Create simulated objects
        self.primary_imu = SimulatedIMU()
        self.secondary_imu = None  # Only simulate primary IMU
    
    def _get_imu_data(self, imu_node):
        """
        Get data from a specific IMU node.
        
        Args:
            imu_node: MSCL InertialNode object
            
        Returns:
            IMUData with values from the IMU
        """
        data = IMUData()
        data.timestamp = time.time()
        
        try:
            # Get data packets
            packets = imu_node.getDataPackets(0)
            
            if packets:
                # Process the most recent packet
                for data_point in packets[-1].data():
                    # Euler angles
                    if data_point.field() == mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES:
                        if data_point.qualifier() == 6:  # Roll
                            data.roll = data_point.as_double()
                        elif data_point.qualifier() == 7:  # Pitch
                            data.pitch = data_point.as_double()
                        elif data_point.qualifier() == 8:  # Yaw
                            data.yaw = data_point.as_double()
                    
                    # Acceleration
                    elif data_point.field() == mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC:
                        if data_point.qualifier() == 1:  # X
                            data.accel_x = data_point.as_double()
                        elif data_point.qualifier() == 2:  # Y
                            data.accel_y = data_point.as_double()
                        elif data_point.qualifier() == 3:  # Z
                            data.accel_z = data_point.as_double()
                    
                    # Gyro
                    elif data_point.field() == mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC:
                        if data_point.qualifier() == 1:  # X
                            data.gyro_x = data_point.as_double()
                        elif data_point.qualifier() == 2:  # Y
                            data.gyro_y = data_point.as_double()
                        elif data_point.qualifier() == 3:  # Z
                            data.gyro_z = data_point.as_double()
            
            # Calculate magnitudes
            data.accel_magnitude = math.sqrt(
                data.accel_x**2 + data.accel_y**2 + data.accel_z**2
            )
            data.gyro_magnitude = math.sqrt(
                data.gyro_x**2 + data.gyro_y**2 + data.gyro_z**2
            )
            
            # Apply high-pass filter to accel_z to detect impacts
            # y[n] = alpha * (y[n-1] + x[n] - x[n-1])
            data.accel_z_highpass = self.highpass_alpha * (
                self.current_data.accel_z_highpass + 
                data.accel_z - 
                self.previous_raw_accel_z
            )
            self.previous_raw_accel_z = data.accel_z
            
        except Exception as e:
            logger.error(f"Error getting IMU data: {e}")
        
        return data
    
    def update_data(self):
        """Update IMU data from all available IMUs"""
        # Save previous data
        self.previous_data = self.current_data
        
        # Get new data - prefer primary IMU
        if self.primary_imu is not None:
            self.current_data = self._get_imu_data(self.primary_imu)
        elif self.secondary_imu is not None:
            self.current_data = self._get_imu_data(self.secondary_imu)
    
    def _update_motion_state(self):
        """
        Update the motion state based on IMU data.
        
        This implements the state machine for detecting:
        - Lowering phase
        - Impact detection
        - Stationary detection
        
        These states are used for the automatic release algorithm.
        """
        current_time = time.time()
        current_state = self.motion_state
        current_data = self.current_data
        
        # Maintain history for stationary detection
        self.stationary_history.append((
            abs(current_data.accel_z - (-GRAVITY)),  # Deviation from gravity
            current_data.gyro_magnitude
        ))
        
        # Keep only the most recent window
        window_samples = int(STATIONARY_WINDOW * self.sampling_rate)
        if len(self.stationary_history) > window_samples:
            self.stationary_history = self.stationary_history[-window_samples:]
        
        # State machine transitions
        if current_state == MotionState.UNKNOWN:
            # Initial state - transition to STATIC
            self.motion_state = MotionState.STATIC
            self.state_entry_time = current_time
        
        elif current_state == MotionState.STATIC:
            # Check for lowering motion (significant -Z acceleration)
            if current_data.accel_z < LOWERING_THRESHOLD:
                self.lowering_start_time = current_time
                self.motion_state = MotionState.LOWERING
                self.state_entry_time = current_time
                logger.debug("State: STATIC -> LOWERING")
        
        elif current_state == MotionState.LOWERING:
            # Check if we're still lowering
            if current_data.accel_z >= LOWERING_THRESHOLD:
                # No longer lowering
                if (current_time - self.state_entry_time) < LOWERING_MIN_DURATION:
                    # Too brief, go back to STATIC
                    self.motion_state = MotionState.STATIC
                    self.state_entry_time = current_time
                    logger.debug("State: LOWERING -> STATIC (too brief)")
            
            # Check for impact (high-pass filtered acceleration spike)
            if abs(current_data.accel_z_highpass) > IMPACT_THRESHOLD:
                self.motion_state = MotionState.IMPACT
                self.state_entry_time = current_time
                self.impact_detection_time = current_time
                logger.debug(f"State: LOWERING -> IMPACT (spike: {current_data.accel_z_highpass:.2f})")
        
        elif current_state == MotionState.IMPACT:
            # Always transition to STATIONARY after impact
            self.motion_state = MotionState.STATIONARY
            self.state_entry_time = current_time
            logger.debug("State: IMPACT -> STATIONARY")
        
        elif current_state == MotionState.STATIONARY:
            # Check if we're still stationary
            if len(self.stationary_history) >= 5:  # Need enough samples
                # Check if all samples in the window are stationary
                is_stationary = all(
                    accel_dev < ACCEL_STATIONARY_THRESHOLD and 
                    gyro_mag < ANGULAR_RATE_STATIONARY_THRESHOLD
                    for accel_dev, gyro_mag in self.stationary_history
                )
                
                # If not stationary, go back to STATIC
                if not is_stationary:
                    self.motion_state = MotionState.STATIC
                    self.state_entry_time = current_time
                    logger.debug("State: STATIONARY -> STATIC (movement detected)")
            
            # Check duration - if we've been stationary long enough after impact
            stationary_duration = current_time - self.impact_detection_time
            if stationary_duration >= STATIONARY_WINDOW:
                # We maintain STATIONARY state but signal the duration is sufficient
                pass
    
    def get_motion_state(self):
        """
        Get the current motion state.
        
        Returns:
            Tuple of (MotionState, duration, elapsed_since_impact)
        """
        current_time = time.time()
        duration = current_time - self.state_entry_time
        elapsed_since_impact = current_time - self.impact_detection_time
        
        return (self.motion_state, duration, elapsed_since_impact)
    
    def is_release_condition(self):
        """
        Check if release condition is met.
        
        This method checks if we have:
        1. Detected an impact
        2. Been stationary for at least STATIONARY_WINDOW seconds
        
        Returns:
            True if release condition is met, False otherwise
        """
        state, duration, elapsed_since_impact = self.get_motion_state()
        
        if (state == MotionState.STATIONARY and 
            elapsed_since_impact >= STATIONARY_WINDOW):
            return True
        
        return False
    
    def _imu_update_thread(self):
        """Thread for continuous IMU data updates"""
        next_update_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next update
            if current_time >= next_update_time:
                # Update IMU data
                self.update_data()
                
                # Update motion state
                self._update_motion_state()
                
                # Calculate next update time
                next_update_time = current_time + self.sampling_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def start(self):
        """Start the IMU interface"""
        if not self.running:
            # Only start if we have at least one IMU
            if self.primary_imu is not None or self.secondary_imu is not None:
                self.running = True
                self.thread = threading.Thread(target=self._imu_update_thread)
                self.thread.daemon = True
                self.thread.start()
                logger.info("IMU interface started")
            else:
                logger.error("Cannot start IMU interface - no IMUs available")
    
    def stop(self):
        """Stop the IMU interface"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        logger.info("IMU interface stopped")
    
    def get_data(self):
        """
        Get the latest IMU data.
        
        Returns:
            Copy of the current IMUData
        """
        return self.current_data

# For testing
if __name__ == "__main__":
    print("Testing IMU interface...")
    
    # Force simulated mode for testing
    imu = IMUInterface()
    imu.start()
    
    try:
        end_time = time.time() + 10  # Run for 10 seconds
        while time.time() < end_time:
            data = imu.get_data()
            state, duration, elapsed = imu.get_motion_state()
            
            print(f"Roll: {data.roll:.1f}°, Pitch: {data.pitch:.1f}°, Yaw: {data.yaw:.1f}°")
            print(f"Accel: X={data.accel_x:.2f}, Y={data.accel_y:.2f}, Z={data.accel_z:.2f} m/s²")
            print(f"Gyro: X={data.gyro_x:.1f}, Y={data.gyro_y:.1f}, Z={data.gyro_z:.1f} °/s")
            print(f"Motion state: {state.name}, Duration: {duration:.2f}s")
            print(f"Release condition: {imu.is_release_condition()}\n")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("Test interrupted")
    
    finally:
        imu.stop()
        print("Test complete")