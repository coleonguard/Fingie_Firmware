#!/usr/bin/env python3
"""
Motor Testing Tool.

This tool tests connections to the Ability Hand motors and provides utilities
for controlling individual fingers and testing motor functionality.
"""

import os
import sys
import time
import argparse
import logging
import threading
import signal
import json
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("MotorTester")

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import necessary interfaces
try:
    from hand.ability_hand_interface import AbilityHandInterface
    from hand.motor_interface import MotorInterface, ControlMode
except ImportError as e:
    logger.error(f"Failed to import motor interfaces: {e}")
    logger.error("Make sure you are running from the project root directory")
    sys.exit(1)

# Check for plotting support
try:
    import matplotlib.pyplot as plt
    import numpy as np
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    logger.warning("Matplotlib not available. Install with 'pip install matplotlib' for plotting support.")

# Constants
DEFAULT_PORT = None  # Let AbilityHandInterface find the port
DEFAULT_BAUD = 460800
DEFAULT_RATE = 50
DEFAULT_DURATION = 5  # seconds
FINGER_MAP = {
    0: "Thumb",
    1: "Index",
    2: "Middle",
    3: "Ring",
    4: "Pinky",
    5: "ThumbRotate"
}
FINGER_NAMES = list(FINGER_MAP.values())

class MotorTester:
    """
    Tester for Ability Hand motors.
    
    This class provides utilities for testing motor connections, controlling
    individual fingers, and running test sequences.
    """
    
    def __init__(
        self,
        port=DEFAULT_PORT,
        baud=DEFAULT_BAUD,
        control_rate=DEFAULT_RATE,
        finger_index=None
    ):
        """
        Initialize the motor tester.
        
        Args:
            port: Serial port for the Ability Hand
            baud: Baud rate
            control_rate: Control rate in Hz
            finger_index: Index of finger to test (0-5), or None for all
        """
        self.port = port
        self.baud = baud
        self.control_rate = control_rate
        self.finger_index = finger_index
        
        # Data for plotting
        self.timestamps = []
        self.position_data = {finger: [] for finger in FINGER_NAMES}
        self.current_data = {finger: [] for finger in FINGER_NAMES}
        self.velocity_data = {finger: [] for finger in FINGER_NAMES}
        
        # Running state
        self.running = False
        self.thread = None
        
        # Display configuration
        logger.info(f"Motor Tester Configuration:")
        logger.info(f"  Port:       {self.port if self.port else 'Auto-detect'}")
        logger.info(f"  Baud Rate:  {self.baud}")
        logger.info(f"  Rate:       {self.control_rate} Hz")
        if finger_index is not None:
            finger_name = FINGER_MAP.get(finger_index, f"Unknown ({finger_index})")
            logger.info(f"  Finger:     {finger_name}")
        
        # Initialize hand interface
        try:
            self.hand = AbilityHandInterface(
                control_rate=self.control_rate,
                port=self.port,
                baud_rate=self.baud,
                reply_mode=2  # Position, Current, Velocity
            )
            logger.info("Hand interface initialized")
        except Exception as e:
            logger.error(f"Failed to initialize hand interface: {e}")
            sys.exit(1)
    
    def _get_finger_from_index(self, index):
        """Get finger name from index"""
        if index is None:
            return None
        return FINGER_MAP.get(index, None)
    
    def _get_index_from_finger(self, finger):
        """Get index from finger name"""
        for idx, name in FINGER_MAP.items():
            if name == finger:
                return idx
        return None
    
    def _update_data_buffers(self):
        """Update data buffers for plotting"""
        timestamp = time.time()
        self.timestamps.append(timestamp)
        
        # Add data for all fingers to buffers
        for finger in FINGER_NAMES:
            finger_status = self.hand.get_finger_status(finger)
            self.position_data[finger].append(finger_status['position'])
            self.current_data[finger].append(finger_status['current'])
            self.velocity_data[finger].append(finger_status['velocity'])
        
        # Limit buffer size to 500 samples
        if len(self.timestamps) > 500:
            self.timestamps = self.timestamps[-500:]
            for finger in FINGER_NAMES:
                self.position_data[finger] = self.position_data[finger][-500:]
                self.current_data[finger] = self.current_data[finger][-500:]
                self.velocity_data[finger] = self.velocity_data[finger][-500:]
    
    def show_status(self):
        """Show current status of all motors"""
        if not self.hand:
            logger.error("Hand interface not initialized")
            return
        
        # Start the hand interface if not running
        if not self.hand.running:
            self.hand.start()
            time.sleep(0.5)  # Give it time to start
        
        try:
            # Collect status for each finger
            status = {}
            for finger in FINGER_NAMES:
                status[finger] = self.hand.get_finger_status(finger)
            
            # Format and display
            print("\n========== Ability Hand Motor Status ==========")
            print(f"{'Finger':<12} {'Position':<10} {'Current':<10} {'Velocity':<10} {'Mode':<10}")
            print("-" * 55)
            
            for finger in FINGER_NAMES:
                finger_status = status[finger]
                print(f"{finger:<12} {finger_status['position']:7.1f}°   {finger_status['current']:6.3f}A   "
                      f"{finger_status['velocity']:6.1f}°/s   {finger_status['mode']:<10}")
            
            # Add fault status
            fault_status = self.hand.get_fault_status()
            print("\nFault Status:")
            for fault, status in fault_status.items():
                status_str = "FAULT" if status else "OK"
                print(f"  {fault}: {status_str}")
            
            print("\n==============================================")
            
            return status
            
        except Exception as e:
            logger.error(f"Failed to get status: {e}")
            return None
    
    def run_position_command(self, finger_index, position, duration=DEFAULT_DURATION, plot=False):
        """
        Run a position command on a specific finger.
        
        Args:
            finger_index: Index of finger (0-5)
            position: Target position (0-100 degrees)
            duration: Duration to hold position (seconds)
            plot: Whether to plot results
        """
        finger = self._get_finger_from_index(finger_index)
        if not finger:
            logger.error(f"Invalid finger index: {finger_index}")
            return
        
        logger.info(f"Setting {finger} to position {position:.1f}° for {duration:.1f}s")
        
        # Start the hand interface
        if not self.hand.running:
            self.hand.start()
        
        # Clear data buffers
        self.timestamps = []
        for finger_name in FINGER_NAMES:
            self.position_data[finger_name] = []
            self.current_data[finger_name] = []
            self.velocity_data[finger_name] = []
        
        try:
            # Apply position command
            self.hand.set_position(finger, position)
            
            # Monitor for the specified duration
            start_time = time.time()
            end_time = start_time + duration
            
            # Create plot if requested
            if plot and PLOTTING_AVAILABLE:
                plt.ion()  # Interactive mode
                fig, axes = plt.subplots(3, 1, figsize=(10, 8))
                fig.suptitle(f"{finger} Position Control Test")
                
                # Position plot
                position_line, = axes[0].plot([], [], 'b-', label=finger)
                axes[0].set_ylabel('Position (°)')
                axes[0].set_title('Position')
                axes[0].grid(True)
                
                # Current plot
                current_line, = axes[1].plot([], [], 'r-', label=finger)
                axes[1].set_ylabel('Current (A)')
                axes[1].set_title('Current')
                axes[1].grid(True)
                
                # Velocity plot
                velocity_line, = axes[2].plot([], [], 'g-', label=finger)
                axes[2].set_xlabel('Time (s)')
                axes[2].set_ylabel('Velocity (°/s)')
                axes[2].set_title('Velocity')
                axes[2].grid(True)
                
                plt.tight_layout()
                plt.subplots_adjust(top=0.9)
            
            # Main monitoring loop
            last_plot_time = 0
            plot_interval = 0.1  # Update plot every 100ms
            
            while time.time() < end_time and self.hand.running:
                # Update data
                self._update_data_buffers()
                
                # Show current data
                status = self.hand.get_finger_status(finger)
                os.system('clear' if os.name == 'posix' else 'cls')
                print(f"\n===== {finger} Position Command =====")
                print(f"Target:    {position:.1f}°")
                print(f"Current:   {status['position']:.1f}°")
                print(f"Error:     {position - status['position']:.1f}°")
                print(f"Current:   {status['current']:.3f}A")
                print(f"Velocity:  {status['velocity']:.1f}°/s")
                print(f"Elapsed:   {time.time() - start_time:.1f}s / {duration:.1f}s")
                print("\nPress Ctrl+C to stop")
                
                # Update plot if enough time has passed
                if plot and PLOTTING_AVAILABLE and time.time() - last_plot_time > plot_interval:
                    self._update_position_plot(
                        (position_line, current_line, velocity_line),
                        axes, fig, finger, start_time
                    )
                    last_plot_time = time.time()
                
                # Sleep to maintain control rate
                time.sleep(1.0 / self.control_rate)
            
            # Final update
            if plot and PLOTTING_AVAILABLE:
                self._update_position_plot(
                    (position_line, current_line, velocity_line),
                    axes, fig, finger, start_time
                )
            
        except KeyboardInterrupt:
            logger.info("Command interrupted by user")
        except Exception as e:
            logger.error(f"Error in position command: {e}")
        
        # Wait a moment before stopping plotting
        if plot and PLOTTING_AVAILABLE:
            plt.ioff()
            print("\nClose the plot window to continue...")
            plt.show(block=True)
    
    def _update_position_plot(self, lines, axes, fig, finger, start_time):
        """Update the plot with new data for position control"""
        if len(self.timestamps) < 2:
            return
        
        # Calculate relative timestamps
        rel_timestamps = [t - start_time for t in self.timestamps]
        
        # Update plots
        position_line, current_line, velocity_line = lines
        
        position_line.set_data(rel_timestamps, self.position_data[finger])
        current_line.set_data(rel_timestamps, self.current_data[finger])
        velocity_line.set_data(rel_timestamps, self.velocity_data[finger])
        
        # Adjust limits
        for i, ax in enumerate(axes):
            ax.relim()
            ax.autoscale_view()
            
            # Set x-axis limits
            ax.set_xlim(0, max(rel_timestamps))
        
        # Draw
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    
    def run_current_command(self, finger_index, current, duration=DEFAULT_DURATION, plot=False):
        """
        Run a current (torque) command on a specific finger.
        
        Args:
            finger_index: Index of finger (0-5)
            current: Target current (0-0.6 A)
            duration: Duration to apply current (seconds)
            plot: Whether to plot results
        """
        finger = self._get_finger_from_index(finger_index)
        if not finger:
            logger.error(f"Invalid finger index: {finger_index}")
            return
        
        logger.info(f"Setting {finger} to current {current:.3f}A for {duration:.1f}s")
        
        # Start the hand interface
        if not self.hand.running:
            self.hand.start()
        
        # Clear data buffers
        self.timestamps = []
        for finger_name in FINGER_NAMES:
            self.position_data[finger_name] = []
            self.current_data[finger_name] = []
            self.velocity_data[finger_name] = []
        
        try:
            # Apply current command
            self.hand.set_torque(finger, current)
            
            # Monitor for the specified duration
            start_time = time.time()
            end_time = start_time + duration
            
            # Create plot if requested
            if plot and PLOTTING_AVAILABLE:
                plt.ion()  # Interactive mode
                fig, axes = plt.subplots(3, 1, figsize=(10, 8))
                fig.suptitle(f"{finger} Current Control Test")
                
                # Position plot
                position_line, = axes[0].plot([], [], 'b-', label=finger)
                axes[0].set_ylabel('Position (°)')
                axes[0].set_title('Position')
                axes[0].grid(True)
                
                # Current plot
                current_line, = axes[1].plot([], [], 'r-', label=finger)
                axes[1].set_ylabel('Current (A)')
                axes[1].set_title('Current')
                axes[1].grid(True)
                
                # Velocity plot
                velocity_line, = axes[2].plot([], [], 'g-', label=finger)
                axes[2].set_xlabel('Time (s)')
                axes[2].set_ylabel('Velocity (°/s)')
                axes[2].set_title('Velocity')
                axes[2].grid(True)
                
                plt.tight_layout()
                plt.subplots_adjust(top=0.9)
            
            # Main monitoring loop
            last_plot_time = 0
            plot_interval = 0.1  # Update plot every 100ms
            
            while time.time() < end_time and self.hand.running:
                # Update data
                self._update_data_buffers()
                
                # Show current data
                status = self.hand.get_finger_status(finger)
                os.system('clear' if os.name == 'posix' else 'cls')
                print(f"\n===== {finger} Current Command =====")
                print(f"Target:    {current:.3f}A")
                print(f"Current:   {status['current']:.3f}A")
                print(f"Error:     {current - status['current']:.3f}A")
                print(f"Position:  {status['position']:.1f}°")
                print(f"Velocity:  {status['velocity']:.1f}°/s")
                print(f"Elapsed:   {time.time() - start_time:.1f}s / {duration:.1f}s")
                print("\nPress Ctrl+C to stop")
                
                # Update plot if enough time has passed
                if plot and PLOTTING_AVAILABLE and time.time() - last_plot_time > plot_interval:
                    self._update_position_plot(
                        (position_line, current_line, velocity_line),
                        axes, fig, finger, start_time
                    )
                    last_plot_time = time.time()
                
                # Sleep to maintain control rate
                time.sleep(1.0 / self.control_rate)
            
            # Final update
            if plot and PLOTTING_AVAILABLE:
                self._update_position_plot(
                    (position_line, current_line, velocity_line),
                    axes, fig, finger, start_time
                )
            
        except KeyboardInterrupt:
            logger.info("Command interrupted by user")
        except Exception as e:
            logger.error(f"Error in current command: {e}")
        
        # Wait a moment before stopping plotting
        if plot and PLOTTING_AVAILABLE:
            plt.ioff()
            print("\nClose the plot window to continue...")
            plt.show(block=True)
    
    def run_test_sequence(self, plot=False):
        """
        Run a comprehensive test sequence on all fingers.
        
        Args:
            plot: Whether to plot results
        """
        logger.info("Running test sequence on all fingers")
        
        # Start the hand interface
        if not self.hand.running:
            self.hand.start()
        
        try:
            # Step 1: Zero all fingers
            logger.info("Step 1: Zeroing all fingers")
            for finger in FINGER_NAMES:
                self.hand.set_position(finger, 0.0)
            time.sleep(2.0)
            
            # Step 2: Position test - sequential finger movement
            logger.info("Step 2: Sequential finger movement")
            for finger_index in range(6):
                finger = self._get_finger_from_index(finger_index)
                if finger:
                    logger.info(f"Moving {finger} to 50°")
                    self.hand.set_position(finger, 50.0)
                    time.sleep(1.0)
                    self.hand.set_position(finger, 0.0)
                    time.sleep(0.5)
            
            # Step 3: All fingers together
            logger.info("Step 3: All fingers together")
            for position in [30.0, 60.0, 90.0, 0.0]:
                logger.info(f"Moving all fingers to {position}°")
                for finger in self.hand.fingers:
                    self.hand.set_position(finger, position)
                
                # Special handling for thumb rotation
                if position == 0.0:
                    self.hand.set_position("ThumbRotate", 0.0)
                else:
                    self.hand.set_position("ThumbRotate", position / 2)
                
                time.sleep(1.5)
            
            # Step 4: Current control test on index finger
            logger.info("Step 4: Current control test on index finger")
            self.hand.set_position("Index", 50.0)
            time.sleep(1.0)
            
            # Apply increasing current
            for current in [0.1, 0.2, 0.3, 0.0]:
                logger.info(f"Setting Index finger current to {current:.1f}A")
                self.hand.set_torque("Index", current)
                time.sleep(1.5)
            
            # Step 5: Reset all to zero
            logger.info("Step 5: Resetting all fingers")
            for finger in FINGER_NAMES:
                self.hand.set_position(finger, 0.0)
            
            logger.info("Test sequence completed successfully")
            
        except KeyboardInterrupt:
            logger.info("Test sequence interrupted by user")
        except Exception as e:
            logger.error(f"Error in test sequence: {e}")
    
    def save_data_to_file(self, filename=None):
        """Save the collected data to a file"""
        if not self.timestamps:
            logger.warning("No data to save")
            return
        
        # Generate filename with timestamp if not provided
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"motor_data_{timestamp}.json"
        
        # Create data object
        data_object = {
            "timestamp": datetime.now().isoformat(),
            "device_info": {
                "port": self.port,
                "baud_rate": self.baud,
                "control_rate": self.control_rate
            },
            "samples": {
                "timestamps": self.timestamps,
                "position": {finger: self.position_data[finger] for finger in FINGER_NAMES},
                "current": {finger: self.current_data[finger] for finger in FINGER_NAMES},
                "velocity": {finger: self.velocity_data[finger] for finger in FINGER_NAMES}
            }
        }
        
        # Write to file
        try:
            with open(filename, 'w') as f:
                json.dump(data_object, f, indent=2)
            logger.info(f"Data saved to {filename}")
            return filename
        except Exception as e:
            logger.error(f"Failed to save data: {e}")
            return None
    
    def start(self):
        """Start the hand interface"""
        if not self.hand.running:
            self.hand.start()
    
    def stop(self):
        """Stop the hand interface"""
        if self.hand and self.hand.running:
            self.hand.stop()
            logger.info("Hand interface stopped")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Motor Testing Tool")
    
    # Connection options
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f"Serial port of Ability Hand (default: auto-detect)")
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    
    # Mode options
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--status', action='store_true',
                           help="Show motor status")
    mode_group.add_argument('--position', type=float,
                           help="Set position (0-100 degrees)")
    mode_group.add_argument('--current', type=float,
                           help="Set current (0-0.6 A)")
    mode_group.add_argument('--sequence', action='store_true',
                           help="Run test sequence")
    
    # Finger selection
    parser.add_argument('--finger', type=int, choices=range(6),
                       help="Finger index (0: Thumb, 1: Index, 2: Middle, 3: Ring, 4: Pinky, 5: Thumb Rotate)")
    
    # Other options
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION,
                       help=f"Duration in seconds (default: {DEFAULT_DURATION})")
    parser.add_argument('--plot', action='store_true',
                       help="Show real-time plot (requires matplotlib)")
    parser.add_argument('--save', type=str, default=None,
                       help="Save data to specified file after command")
    
    args = parser.parse_args()
    
    # Create tester
    tester = MotorTester(
        port=args.port,
        baud=args.baud,
        finger_index=args.finger
    )
    
    # Handle interrupts gracefully
    def signal_handler(sig, frame):
        print("\nExiting...")
        tester.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Run requested mode
        if args.status:
            tester.show_status()
        elif args.position is not None:
            if args.finger is None:
                logger.error("Finger index is required for position control")
                sys.exit(1)
            tester.run_position_command(args.finger, args.position, args.duration, args.plot)
        elif args.current is not None:
            if args.finger is None:
                logger.error("Finger index is required for current control")
                sys.exit(1)
            tester.run_current_command(args.finger, args.current, args.duration, args.plot)
        elif args.sequence:
            tester.run_test_sequence(args.plot)
        else:
            # Default: show status
            tester.show_status()
            
        # Save data if requested
        if args.save and len(tester.timestamps) > 0:
            tester.save_data_to_file(args.save)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        tester.stop()

if __name__ == "__main__":
    main()