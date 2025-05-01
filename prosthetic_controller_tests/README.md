# Unified Prosthetic Hand Control System

This module provides a unified control framework for commanding hand movements through proximity sensors and receiving position/torque/current feedback in a specific control loop frequency.

## Overview

The system integrates:

1. **Proximity Sensor Input**: Uses VL6180X proximity sensors connected through I²C multiplexers to detect object proximity
2. **Control Logic**: Transforms proximity readings into appropriate motor commands
3. **Motor Control Interface**: Controls physical hand motors via position, torque, or current commands
4. **Feedback Loop**: Reads back position, torque, and current data at configurable frequency

## Hardware Requirements

- Raspberry Pi (3B+ or newer)
- 2× TCA9548A I²C multiplexers (addresses 0x70, 0x71)
- 10× VL6180X proximity sensors
- Prosthetic hand with motor drivers
- USB serial connection to hand control board

## System Architecture

The system follows a modular design:

```
ProximityManager → ControlSystem → MotorInterface
                        ↑               ↓
                        └───── Feedback ←┘
```

1. **ProximityManager**: Handles sensor initialization, reading, and filtering
2. **ControlSystem**: Implements control algorithms (proportional, PID, etc.)
3. **MotorInterface**: Provides a unified API for different motor control methods
4. **FeedbackSystem**: Processes position/current/torque feedback from motors

## Key Features

- **Configurable control loop rate** (10-100Hz)
- **Multiple control modes** (position, torque, hybrid)
- **Real-time sensor filtering** with Kalman filters
- **Comprehensive data logging** for analysis and debugging
- **Extensible interface** for different hand hardware

## Usage Example

```python
from prosthetic_control.unified_controller import UnifiedController

# Create controller with 20Hz control loop
controller = UnifiedController(control_rate=20)

# Start the control system
controller.start()

# System runs automatically based on proximity readings
# Get status information as needed
finger_status = controller.get_finger_status("index")
print(f"Finger position: {finger_status['position']}")
print(f"Motor current: {finger_status['current']}")

# Stop the system when done
controller.stop()
```

## Configuration

The system can be configured through:

1. **Control parameters**: Thresholds, gains, control rates
2. **Sensor sensitivity**: Filtering parameters, proximity thresholds
3. **Motor limits**: Current/torque safety limits

See `config.py` for detailed configuration options.