# Prosthetic Control System

This directory contains the unified prosthetic-hand controller implementation based on the system development plan described in `system_dev_plan.md`.

## Overview

The prosthetic control system implements a complete reach → grasp → lift → lower → release controller with proximity-based finger control and IMU-based object release detection. The system integrates:

- VL6180X proximity sensors for finger control
- Microstrain IMUs for hand orientation and movement detection
- Ability Hand motor interface for position and torque control
- Unified controller implementing the control algorithms and state machines

## Implementation Progress

- [x] Create project directory structure
- [x] Implement proximity sensor interface with Kalman filtering
- [x] Implement IMU interface with acceleration and angular rate processing
- [x] Implement hand motor interface for position and torque control
- [x] Implement finger finite state machine (APPROACH/PROPORTIONAL/CONTACT)
- [x] Implement hand finite state machine (REACH/LIFT/LOWER/RELEASE/RETRACT)
- [x] Implement unified controller with safety caps and logging
- [x] Implement mock classes for testing
- [x] Implement basic unit tests
- [x] Implement component tests
  - [x] U-01: Kalman glitch mask
  - [x] U-02: Velocity limiter 
  - [x] U-03: Torque clamp
  - [x] U-04: IMU sign self-test
  - [x] C-01: Finger θ monotonic
  - [x] C-02: dI/dt contact trigger
  - [x] C-03: Hand FSM release
  - [x] C-04: Watchdog trip
  - [x] I-01: 20 Hz timing
  - [x] I-02: Log schema
  - [ ] I-03: Deterministic replay
  - [ ] I-04: Thread safety
  - [ ] S-01: Monte-Carlo 100 grasps
- [ ] Validate against all acceptance criteria

## Getting Started

### Prerequisites

- Python 3.10 or higher
- SMBus2 library for I2C communication with sensors
- MSCL library for Microstrain IMUs (optional, will use simulation mode if not available)
- Ability Hand API for prosthetic control (optional, will use simulation mode if not available)

### Installation

1. Clone the repository
2. Install dependencies
3. Run the controller

```bash
python main.py
```

### Command Line Options

The `main.py` script accepts the following command-line options:

- `--simulate`: Use simulated hardware instead of real devices
- `--rate <hz>`: Control loop rate in Hz (default: 20)
- `--no-logging`: Disable data logging
- `--log-dir <path>`: Directory for log files (default: ~/prosthetic_logs)
- `--port <port>`: Serial port for Ability Hand (default: auto-detect)

## Directory Structure

- `proximity/` - Proximity sensor management and filtering
  - `proximity_manager.py` - Interface to the VL6180X sensors
  - `kalman_filter.py` - Kalman filter for sensor smoothing
- `imu/` - IMU interface and processing
  - `imu_interface.py` - Interface to the Microstrain IMUs
- `hand/` - Hand motor interface and control
  - `motor_interface.py` - Abstract interface for motor control
  - `ability_hand_interface.py` - Interface for the Ability Hand
- `controller/` - Unified controller and state machines
  - `state_machines.py` - Finger and hand state machines
  - `unified_controller.py` - Main controller integrating all components
- `mocks/` - Mock objects for testing
  - `mock_components.py` - Mock implementations of hardware components
- `tests/` - Test suite
  - `test_kalman_filter.py` - Unit tests for the Kalman filter
- `utils/` - Utility functions and helpers
  - `logger.py` - Data logging utility

## Technical Details

### Distance → Position Mapping

When a finger's proximity sensor detects an object within the approach threshold (40mm), the controller commands the finger to close proportionally based on the distance:

```
θ_f = 0                                  if d_f ≥ 40mm (approach threshold)
θ_f = θ_max * (40-d_f)/(40-5)            if 5mm < d_f < 40mm
θ_f = θ_max = 100°                       if d_f ≤ 5mm (contact threshold)
```

Position changes are rate-limited to 8° per control cycle (50ms).

### Contact-phase Torque Regulation

When contact is detected (distance ≤ 5mm or current derivative threshold), the finger switches to torque control:
- Initial torque is set to 0.30A
- If current exceeds 0.45A, torque is reduced to 90% of current value
- If slip is detected (current derivative < -0.05A/s for ≥2 cycles), torque is increased by 0.05A (up to 0.6A max)

### IMU-based Table Contact Detection

The system detects table contact and object release using the IMU:
- Lowering phase starts when az < g-4 m/s² for ≥50ms
- Impact impulse is detected when high-pass filtered az > 7.8 m/s²
- Stationary state is detected when az-g < 1 m/s² and |ω| < 10°/s for 0.3s
- Release is triggered when the hand is stationary and all finger currents < 0.2A

## Testing

The system includes a comprehensive test suite to verify functionality:

```bash
# Run all tests
python -m unittest discover tests

# Run a specific test
python -m unittest tests.test_kalman_filter
```

### Mock Components

For testing without hardware, the system provides mock implementations of:
- Proximity sensors with configurable noise and failure rates
- IMUs with data replay capability
- Hand motors with simulated dynamics

## Logging

The system logs data in newline-delimited JSON (ND-JSON) format. Each log entry includes:
- Timestamp
- Proximity sensor data (raw and filtered)
- IMU data (orientation, acceleration, angular rates)
- Finger states and positions
- Hand state
- Fault conditions

Log files are stored in the specified log directory (default: ~/prosthetic_logs).

## Acknowledgments

This implementation is based on the system development plan created for the prosthetic hand controller project.