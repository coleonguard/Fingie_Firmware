# Proximity-Only Controller

This module provides a controller for the prosthetic hand that operates using only proximity sensors, without requiring IMU data.

## Overview

The proximity-only controller is designed to:

1. Work without any IMU components
2. Use VL6180X proximity sensors to detect object distance
3. Control the Ability Hand prosthetic
4. Implement the same state machine logic for fingers
5. Log and monitor system performance

## Controller Implementations

This package includes three different controller implementations:

### 1. Original Threaded Controller
The `ProximityController` (default) uses a threaded architecture where:
- A separate background thread continuously reads from proximity sensors
- The main thread processes control logic and sends motor commands
- Thread synchronization is used to coordinate these activities

### 2. Simplified Controller (Reliable)
The `SimplifiedController` uses a single-loop architecture inspired by `fallback_test.py`:
- All operations occur in a single thread in a deterministic, sequential order
- Sensor reading and motor control are strictly separated
- No thread synchronization overhead or context switching
- More reliable I2C communication with proximity sensors
- May produce jerky movements due to direct sensor-to-motor mapping

Use the `--simplified` flag to enable this controller:
```bash
python -m prosthetic_control_system.controller.without_imu.run_controller --simplified
```

### 3. Smooth Controller (Recommended)
The `SmoothController` combines reliability with physics-based smoothing:
- Uses separate threads for sensor reading and motor control
- Sensor thread follows the exact pattern from fallback_test.py for reliable I2C
- Motor thread applies physics-based smoothing for natural movements
- Implements velocity and acceleration limits for smooth transitions
- Eliminates jerky movements while maintaining reliable operation

Use the `--smooth` flag to enable this controller:
```bash
python -m prosthetic_control_system.controller.without_imu.run_controller --smooth
```

## Files

- `config.py` - Configuration parameters for the proximity-only controller
- `proximity_controller.py` - Original threaded controller implementation
- `simplified_controller.py` - Simplified single-loop controller implementation
- `smooth_controller.py` - Physics-based smooth motion controller
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
# Use simplified architecture (recommended for reliability)
python -m prosthetic_control_system.controller.without_imu.run_controller --simplified

# Use smooth controller with physics-based motion (recommended)
python -m prosthetic_control_system.controller.without_imu.run_controller --smooth

# Change control rate
python -m prosthetic_control_system.controller.without_imu.run_controller --rate 30

# Adjust proximity thresholds (in mm)
python -m prosthetic_control_system.controller.without_imu.run_controller --approach 60 --contact 10

# Specify Ability Hand serial port
python -m prosthetic_control_system.controller.without_imu.run_controller --port /dev/ttyUSB0

# Enable/disable logging
python -m prosthetic_control_system.controller.without_imu.run_controller --no-logging

# Run in calibration mode (no motor movements)
python -m prosthetic_control_system.controller.without_imu.run_controller --calibrate

# Run sensor analysis to compare MCP vs PIP sensors
python -m prosthetic_control_system.controller.without_imu.run_controller --analyze-sensors

# Change visualization mode (minimal, basic, detailed)
python -m prosthetic_control_system.controller.without_imu.run_controller --visualization detailed

# Enable verbose debugging output
python -m prosthetic_control_system.controller.without_imu.run_controller --debug

# Combine options
python -m prosthetic_control_system.controller.without_imu.run_controller --smooth --visualization minimal
```

## Hardware Setup

### Prerequisites

1. VL6180X proximity sensors connected to I2C multiplexers
2. Multiplexers connected to the Raspberry Pi
3. Ability Hand connected via USB

### Hardware Recommendations for Reliable I2C

For optimal reliability with the proximity sensors:

1. **I2C Pull-up Resistors**: 
   - Use 2.2kΩ-4.7kΩ pull-up resistors on SDA and SCL lines
   - Weaker pull-ups can cause reliability issues under load

2. **I2C Bus Speed**:
   - Consider reducing the I2C clock speed for improved reliability
   - Default is typically 100kHz; try 50kHz in high-interference environments

3. **Power Supply**:
   - Use separate power supplies for sensors and motors if possible
   - Add decoupling capacitors (0.1μF) near each sensor's power pins

4. **Cable Shielding**:
   - Use shielded cables for I2C connections in high-noise environments
   - Keep I2C cables away from motor wires and power cables

5. **Ground Plane**:
   - Ensure all components share a common ground reference
   - Use a solid ground plane on PCBs for reduced noise

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

## Safety Features and Fault Handling

The controller includes several safety features to protect the Ability Hand:

1. **Safe Default Position**: The hand always defaults to the open/flat position when:
   - The controller is stopped
   - Errors occur in sensor readings
   - System shutdown is triggered

2. **Safety Reset Command**: Press 'r' during operation to immediately open the hand to a safe position

3. **Error Handling**: Extensive error handling ensures that even if components fail, the hand tries to reach a safe position

4. **Fault Detection**: The controller monitors for these fault conditions:
   - Loop overrun: Control loop takes longer than the specified interval
   - Communication loss: Problems with the motor interface
   - Sensor failure: Multiple critical proximity sensors failing

5. **Sensor Substitution**: If a proximity sensor fails, the system tries to substitute readings from neighboring sensors as defined in the fallback map

6. **Architectural Approaches to Prevent I2C Contention**: The two implementations use different approaches to prevent I2C bus contention between sensor readings and motor control:

   **a) Original Threaded Controller** implements a three-phase control strategy:
   - **Phase 1: HARDWARE_RESET**
     - Disables all multiplexers to reset the I2C bus state
     - Adds a stabilization delay to let the I2C bus recover
     - Ensures a clean state before sensor operations

   - **Phase 2: SENSORS_ONLY**
     - Reads all proximity sensors sequentially
     - No motor control operations during this phase
     - Caches sensor values with timestamps for later use
     - Explicitly disables multiplexers after reading

   - **Phase 3: MOTORS_ONLY**
     - Uses cached sensor data to control motors
     - No sensor reading during this phase
     - Processes finger and hand state machines
     - Controls motor positions and torques

   **b) Simplified Controller** follows the `fallback_test.py` architecture:
   - Uses a single sequential control loop with no threading
   - Reads all sensors first, then processes control logic, then sends motor commands
   - No concurrent operations or thread synchronization
   - Implements the same deterministic reading order as the successful experiment
   - Uses the same timing parameters (small sleeps between operations)
   - Includes the same fallback substitution logic
   - More reliable for I2C communication but produces jerky movements
   
   **c) Smooth Controller** (recommended) combines reliability with smooth motion:
   - Uses separate threads but with careful design to prevent I2C contention:
     - Sensor thread exactly replicates the fallback_test.py pattern at 5Hz
     - Motor control thread runs at 50Hz with physics-based smoothing
   - Implements physics-based motion control:
     - Uses velocity limiting to prevent sudden position jumps
     - Uses acceleration limiting for natural motion transitions
     - Results in smooth, natural finger movements
   - Maintains the reliability of the simplified controller while eliminating jerky movements
   
   The smooth controller provides the best combination of reliability and natural motion.

When faults occur, they are logged and displayed in the status output. The controller will try to maintain operation with degraded performance rather than failing completely.

## Visualization Modes

The controller offers three visualization modes for monitoring system status:

1. **Minimal** (`--visualization minimal`)
   - Shows only essential information
   - Hand state and active fingers
   - Minimal screen updates for constrained environments

2. **Basic** (`--visualization basic`, default)
   - Shows finger states with visual indicators (⚪, 🟡, 🟠, 🔴)
   - Displays sensor distances, motor positions, and status
   - Shows system performance metrics

3. **Detailed** (`--visualization detailed`)
   - Shows comprehensive debugging information
   - Displays sensor status, fallback sources, and analysis
   - Shows motion constraints and timing information
   - Ideal for troubleshooting sensor issues

## Performance Tuning

If the controller performs sluggishly:
1. Try the simplified controller (`--simplified`)
2. Decrease the control rate (e.g., `--rate 15`)
3. Increase the approach threshold (e.g., `--approach 80`)
4. Ensure I2C buses are not overloaded
5. Check for sources of interference
6. Try the minimal visualization mode (`--visualization minimal`)
7. Disable sensor analysis if enabled

## Troubleshooting I2C Issues

If you're experiencing sensor errors ("all retries failed to get_distance()"), especially when motors are moving:

1. **Use the Smooth Controller** (recommended):
   ```bash
   python -m prosthetic_control_system.controller.without_imu.run_controller --smooth
   ```
   This should resolve most I2C issues while providing smooth motion because:
   - The sensor thread exactly replicates the reliable pattern from `fallback_test.py`
   - It separates sensor reading and motor control into different threads with clean boundaries
   - The physics-based smoothing eliminates jerky movements
   - It maintains reliable I2C communication even during fast motion

2. **Alternatively, use the Simplified Controller**:
   ```bash
   python -m prosthetic_control_system.controller.without_imu.run_controller --simplified
   ```
   This will provide the most reliable I2C communication but with jerky movements because:
   - It follows the same architecture as the working `fallback_test.py` experiment
   - It avoids thread synchronization issues that can cause timing problems
   - It has deterministic timing and simplified error handling

3. If still experiencing issues, try reducing the control rate:
   ```bash
   python -m prosthetic_control_system.controller.without_imu.run_controller --simplified --rate 10
   ```

4. For hardware debugging, check:
   - **Pull-up Resistors**: Verify you have appropriate pull-up resistors on the I2C bus
   - **Power Supply**: Ensure stable power for sensors and multiplexers
   - **Cable Length**: Keep I2C cables short and away from noise sources

## Logs

Logs are saved to `~/prosthetic_logs` by default (configurable with `--log-dir`).
The logs include:
- Proximity sensor readings
- Finger positions and currents
- State machine states
- Performance metrics