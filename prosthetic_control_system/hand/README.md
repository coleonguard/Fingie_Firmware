# Hand Motor Control Interface

This module provides an interface for controlling the PSYONIC Ability Hand prosthetic through a consistent API. It supports both real hardware and a simulated mode for development and testing.

## Overview

The system consists of two main components:

1. **MotorInterface** - A base abstract class that defines the interface for motor control
2. **AbilityHandInterface** - A concrete implementation for the PSYONIC Ability Hand

## Ability Hand Integration

This module integrates with the Ability Hand API which is located in the project root at `/ability-hand-api/`. The `__init__.py` file in this directory automatically adds the API to the Python path.

### Key Features

- Real-time position control
- Torque/current control
- Velocity control
- Duty cycle control
- Touch sensor feedback
- Slip detection using current derivatives
- Automatic recovery from connection errors
- Simulated mode when hardware is unavailable

## Usage

### Basic Usage

```python
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface

# Create interface (auto-detects port)
hand = AbilityHandInterface(control_rate=50)

# Start the control loop
hand.start()

# Control fingers with position commands (0-100 degrees)
hand.set_position("Thumb", 50.0)
hand.set_position("Index", 70.0)
hand.set_position("ThumbRotate", 30.0)

# Get finger status
thumb_status = hand.get_finger_status("Thumb")
print(f"Position: {thumb_status['position']}°")
print(f"Current: {thumb_status['current']}A")
print(f"Velocity: {thumb_status['velocity']}°/s")

# Control fingers with torque commands (0-0.6A)
hand.set_torque("Thumb", 0.2)

# Stop the control loop when done
hand.stop()
```

### Control Modes

The interface supports different control modes for each finger:

- **Position** - Control finger position (0-100 degrees)
- **Velocity** - Control finger velocity (degrees per second)
- **Torque** - Control finger torque/current (0-0.6A)
- **Duty** - Control motor duty cycle (-100 to 100%)

### Error Handling

The interface automatically handles connection errors and will attempt to reconnect if the connection is lost. If the Ability Hand API is not available, it will fall back to a simulated mode.

## API Requirements

Before using the real hardware:

1. Enable UART and byte stuffing on the Ability Hand using the PSYONIC App:
   - Scan → SELECT HAND → Gear Icon ⚙️ → Troubleshoot → Developer Mode
   - Issue commands: We16, We46, We47

2. Connect the Ability Hand via USB

3. Set the correct baud rate (default: 460800)

## Testing

Use the tools in `prosthetic_control_system/hardware_tools/` to test the Ability Hand:

```
python -m hardware_tools.test_motor --status
```

## Simulated Mode

When the Ability Hand API or hardware is not available, the interface will automatically fall back to a simulated mode. This allows for development and testing without the physical hardware.

The simulation provides:
- Realistic position, velocity, and current feedback
- Simulated touch sensors
- Appropriate response to control commands

To force simulated mode, create the interface with an invalid port:
```python
hand = AbilityHandInterface(port="SIMULATED")
```