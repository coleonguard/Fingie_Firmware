# Prosthetic Control System Controllers

This directory contains controllers for the prosthetic control system that do not rely on IMU data.

## Quick Start

For complete technical documentation with implementation details, tuning guidelines, and troubleshooting, see [CONTROLLER_DETAILS.md](./CONTROLLER_DETAILS.md).

## Available Controllers

### 1. Safe Thumb Opposition Controller

`safe_thumb_opposition_controller.py` - Primary controller with reliable thumb opposition:

```bash
# Basic usage with simulation
python3 -m prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller --simulate

# With real hardware
python3 -m prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller
```

### 2. Wiggle Approach Controller

`wiggle_approach_controller.py` - Enhanced controller with tactile feedback:

```bash
# Basic usage with simulation
python3 -m prosthetic_control_system.controller.without_imu.wiggle_approach_controller --simulate

# With custom wiggle parameters
python3 -m prosthetic_control_system.controller.without_imu.wiggle_approach_controller --wiggle-amplitude 3 --wiggle-frequency 0.7
```

### 3. Experimental Controller

`run_experiment.py` - Experimental controller with finger twitching:

```bash
# Basic usage with simulation
python3 -m prosthetic_control_system.controller.without_imu.run_experiment --simulate
```

## Key Features

- **Robust sensor handling** with advanced debouncing
- **Proper thumb opposition** using maximum rotation
- **Safe finger movement** with collision prevention
- **I2C bus contention management**
- **Natural finger wiggling** for tactile feedback

## Common Options

- `--simulate`: Use simulated hardware
- `--rate RATE`: Control loop rate in Hz (default: 20)
- `--port PORT`: Serial port (default: auto-detect)
- `--finger-close ANGLE`: Finger closure angle in degrees (default: 30.0)
- `--thumb-oppose ANGLE`: Thumb opposition angle in degrees (default: 100.0)
- `--finger-threshold DIST`: Finger detection threshold in mm (default: 100.0)
- `--thumb-threshold DIST`: Thumb detection threshold in mm (default: 50.0)
- `--debug`: Enable debug logging

For detailed implementation information, technical architecture, and development guidelines, see the [CONTROLLER_DETAILS.md](./CONTROLLER_DETAILS.md) document.