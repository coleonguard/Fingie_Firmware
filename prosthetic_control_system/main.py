#!/usr/bin/env python3
"""
Main entry point for the Prosthetic Control System.

This script runs the unified controller with appropriate configuration
based on command-line arguments.
"""

import argparse
import logging
import time
import signal
import sys
import os
from typing import Dict, List, Optional, Union

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("Main")

# Local imports
from controller.unified_controller import UnifiedController

# Global variables
controller = None

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    if controller:
        logger.info("Stopping controller due to signal...")
        controller.stop()
    sys.exit(0)

def main():
    """Main entry point"""
    global controller
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Prosthetic Control System')
    
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                        help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--no-logging', action='store_true',
                        help='Disable data logging')
    
    parser.add_argument('--log-dir', type=str, default=None,
                        help='Directory for log files (default: ~/prosthetic_logs)')
    
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for Ability Hand (default: auto-detect)')
    
    args = parser.parse_args()
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Create motor interface kwargs if needed
        motor_kwargs = {}
        if args.port:
            motor_kwargs['port'] = args.port
        
        # Create controller
        logger.info(f"Creating controller (simulated: {args.simulate}, rate: {args.rate}Hz)")
        controller = UnifiedController(
            control_rate=args.rate,
            enable_logging=not args.no_logging,
            log_dir=args.log_dir,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Print initial status
        status = controller.get_system_status()
        logger.info(f"Controller running. Hand state: {status['hand']['hand_state']}")
        
        # Monitor loop
        while True:
            try:
                # Print status every 5 seconds
                status = controller.get_system_status()
                
                # Check for faults
                faults = status['faults']
                if any(faults.values()):
                    logger.warning(f"FAULTS DETECTED: {faults}")
                
                # Log cycle time stats
                cycle_time = status['cycle_time']
                logger.info(
                    f"Cycle times - Last: {cycle_time['last']*1000:.1f}ms, "
                    f"Avg: {cycle_time['avg']*1000:.1f}ms, "
                    f"Max: {cycle_time['max']*1000:.1f}ms"
                )
                
                # Sleep for a while
                time.sleep(5.0)
                
            except KeyboardInterrupt:
                break
    
    except Exception as e:
        logger.error(f"Error running controller: {e}")
        if controller:
            controller.stop()
    
    finally:
        # Make sure controller is stopped
        if controller:
            logger.info("Stopping controller...")
            controller.stop()
        
        logger.info("Exiting")

if __name__ == "__main__":
    main()