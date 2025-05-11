# Proximity-Only Controller

This module provides a simplified controller for the prosthetic hand that operates using only proximity sensors, without requiring IMU data.

## Overview

The proximity-only controller is designed to:

1. Work without any IMU components
2. Use VL6180X proximity sensors to detect object distance
3. Control the Ability Hand prosthetic
4. Implement the same state machine logic for fingers
5. Log and monitor system performance

## Files

- `config.py` - Configuration parameters for the proximity-only controller
- `proximity_controller.py` - Main controller implementation
- `run_controller.py` - Launch script for the controller

## Usage

### Basic Operation

To run the controller with default settings:

```bash
python -m prosthetic_control_system.controller.without_imu.run_controller
```

### Simulated Mode

To run in simulation mode (without physical hardware):

```bash
python -m prosthetic_control_system.controller.without_imu.run_controller --simulate
```

### Customizing Parameters

You can customize various parameters:

```bash
# Change control rate
python -m prosthetic_control_system.controller.without_imu.run_controller --rate 30

# Adjust proximity thresholds (in mm)
python -m prosthetic_control_system.controller.without_imu.run_controller --approach 60 --contact 10

# Specify Ability Hand serial port
python -m prosthetic_control_system.controller.without_imu.run_controller --port /dev/ttyUSB0

# Enable/disable logging
python -m prosthetic_control_system.controller.without_imu.run_controller --no-logging
```

## Hardware Setup

### Prerequisites

1. VL6180X proximity sensors connected to I2C multiplexers
2. Multiplexers connected to the Raspberry Pi
3. Ability Hand connected via USB

### Multiplexer Addresses

The default configuration expects:
- First multiplexer (MUX1) at address 0x77
- Second multiplexer (MUX2) at address 0x73

You can modify these in `config.py` if your hardware uses different addresses.

### Sensor-to-Finger Mapping

By default, these proximity sensors control the corresponding fingers:
- Thumb1 → Thumb
- Index1 → Index finger
- Middle1 → Middle finger
- Ring1 → Ring finger
- Pinky1 → Pinky finger

## Testing Steps

Follow these steps to systematically test the system:

1. Test Ability Hand control with `hardware_tools.test_motor`
2. Test proximity sensors with `hardware_tools.test_vl6180x`
3. Run the proximity-only controller in simulation mode
4. Run with actual hardware gradually adding components

## Fault Handling

The controller monitors for these fault conditions:
- Loop overrun: Control loop takes longer than the specified interval
- Communication loss: Problems with the motor interface
- Sensor failure: Multiple critical proximity sensors failing

When faults occur, they are logged and displayed in the status output.

## Performance Tuning

If the controller performs sluggishly:
1. Decrease the control rate (e.g., `--rate 15`)
2. Increase the approach threshold (e.g., `--approach 80`)
3. Ensure I2C buses are not overloaded
4. Check for sources of interference

## Logs

Logs are saved to `~/prosthetic_logs` by default (configurable with `--log-dir`).
The logs include:
- Proximity sensor readings
- Finger positions and currents
- State machine states
- Performance metrics