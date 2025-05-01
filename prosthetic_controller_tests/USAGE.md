# Prosthetic Control Tests Usage Guide

This module provides a comprehensive framework for controlling a prosthetic hand using proximity sensors, with interfaces for position, torque, and current control.

## Installation

1. Ensure you have all required dependencies:
   ```bash
   pip install smbus2 numpy 
   ```

2. For using the Ability Hand interface, you need the ability-hand-api:
   ```bash
   # Clone ability-hand-api repo if you don't have it already
   git clone https://github.com/psyonicinc/ability-hand-api.git
   
   # Install the Python requirements
   cd ability-hand-api/python
   pip install -r requirements.txt
   ```

3. Configure the hand for UART and byte stuffing via the app:
   - Enter `We16`, `We46`, and `We47` commands in the developer mode

## Hardware Setup

### Proximity Sensors
- 10 × VL6180X proximity sensors connected via two TCA9548A I²C multiplexers
- Multiplexer addresses: 0x70 and 0x71
- Sensor mapping:
  ```
  MUX1 (0x70):
    Channel 0 → Thumb1
    Channel 1 → Thumb2
    Channel 2 → Index1
    Channel 3 → Index2
    Channel 4 → Middle1
  
  MUX2 (0x71):
    Channel 0 → Middle2
    Channel 1 → Ring1
    Channel 2 → Ring2
    Channel 3 → Pinky1
    Channel 4 → Pinky2
  ```

### Motor Control
Connect the hand via a USB serial adapter to the Raspberry Pi.

## Command-line Usage

### Test Proximity Sensors
```bash
./run_test.py proximity [--duration SECONDS] [--rate HZ]
```

### Test Motor Interface
```bash
./run_test.py motors [--duration SECONDS] [--simulated] [--rate HZ]
```

### Run Unified Controller
```bash
./run_test.py run [--duration SECONDS] [--simulated] [--real] [--rate HZ] [--log-dir DIR]
```

Options:
- `--duration`: Test duration in seconds (0 for indefinite)
- `--simulated`: Use simulated motors instead of real hardware
- `--real`: Force use of real motors (overrides --simulated)
- `--rate`: Control rate in Hz
- `--log-dir`: Directory for log files

## Using as a Library

### Basic Usage

```python
from prosthetic_control_tests import UnifiedController

# Create controller with 20Hz control loop (uses real hardware if available)
controller = UnifiedController(control_rate=20)

# Start the controller
controller.start()

# Get status information
thumb_status = controller.get_finger_status("Thumb")
print(f"Thumb position: {thumb_status['position']}°")
print(f"Thumb current: {thumb_status['current']}A")

# Modify control thresholds if needed
controller.approach_threshold = 50  # mm
controller.contact_threshold = 8    # mm

# Run for a while
time.sleep(30)

# Stop the controller when done
controller.stop()
```

### Using Only Proximity Sensors

```python
from prosthetic_control_tests import ProximityManager

# Create and start the proximity manager
proximity = ProximityManager(sampling_rate=40)
proximity.start()

# Read sensor values
thumb_distance = proximity.get_sensor_value("Thumb1", filtered=True)
index_distance = proximity.get_sensor_value("Index1", filtered=True)

print(f"Thumb distance: {thumb_distance}mm")
print(f"Index distance: {index_distance}mm")

# Clean up
proximity.stop()
```

### Using Only Motor Control

```python
from prosthetic_control_tests import AbilityHandInterface

# Create and start the motor interface
motors = AbilityHandInterface(control_rate=50)
motors.start()

# Set position targets
motors.set_position("Thumb", 50.0)
motors.set_position("Index", 70.0)

# Read back current values
thumb_pos = motors.get_position("Thumb")
thumb_current = motors.get_current("Thumb")

print(f"Thumb position: {thumb_pos}°, current: {thumb_current}A")

# Switch to torque control
motors.set_torque("Thumb", 0.3)

# Clean up
motors.stop()
```

## Configuration

Edit `config.py` to adjust system parameters:

- `DEFAULT_CONTROL_RATE`: Control loop frequency (Hz)
- `APPROACH_THRESHOLD`: Distance threshold for beginning motion (mm)
- `CONTACT_THRESHOLD`: Distance threshold for contact detection (mm)
- `MAX_CURRENT`: Maximum allowed current (A)
- `ABILITY_HAND_PORT`: Serial port for Ability Hand (None for auto-detection)

## Logging

When enabled, the system logs all sensor readings, motor positions, currents, and control phases. Logs are stored in the configured log directory (default: `~/prosthetic_logs/`) as JSON files.

Example log format:
```
{
  "timestamps": [...],
  "proximity": {
    "Thumb1": [...],
    "Index1": [...],
    ...
  },
  "positions": {
    "Thumb": [...],
    "Index": [...],
    ...
  },
  "currents": {...},
  "control_phases": {...}
}
```

These logs can be analyzed with standard Python data analysis tools or visualization software.