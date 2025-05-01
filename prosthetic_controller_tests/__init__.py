"""
Prosthetic Control Tests Module
==============================

This module provides a unified framework for controlling prosthetic hands
using proximity sensors to command joint positions/torques/currents 
with configurable feedback loops.

The main components are:
- proximity_manager.py: Interface to VL6180X proximity sensors via I2C multiplexers
- motor_interface.py: Abstract interface for motor control
- ability_hand_interface.py: Implementation for the PSYONIC Ability Hand
- unified_controller.py: Integration of proximity sensing and motor control
- config.py: Configuration parameters
- run_test.py: Command-line test runner

Usage:
  ./run_test.py proximity  # Test proximity sensors only
  ./run_test.py motors     # Test motor interface only
  ./run_test.py run        # Run the full unified controller
"""

from .proximity_manager import ProximityManager
from .motor_interface import MotorInterface, SimulatedMotorInterface, ControlMode
from .ability_hand_interface import AbilityHandInterface
from .unified_controller import UnifiedController, ControlPhase