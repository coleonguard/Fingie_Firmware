# Prosthetic Control System Controllers

This directory contains controllers for the prosthetic control system that do not rely on IMU data.

## Controllers

### 1. Simple Opposition Controller

`simple_opposition_controller.py` implements a basic, robust control approach that:

1. Rotates the thumb to opposition position when ANY finger detects an object
2. Closes the other fingers when the thumb detects an object
3. Handles I2C bus contention by separating sensor reading and motor control phases

#### Usage

```bash
python3 -m prosthetic_control_system.controller.without_imu.simple_opposition_controller [options]
```

Options:
- `--simulate`: Use simulated hardware
- `--rate RATE`: Control loop rate in Hz (default: 20)
- `--port PORT`: Serial port for Ability Hand (default: auto-detect)
- `--proximity-rate PROXIMITY_RATE`: Proximity sensor sampling rate in Hz (default: 5)
- `--finger-close FINGER_CLOSE`: Finger closure angle in degrees (default: 30.0)
- `--thumb-oppose THUMB_OPPOSE`: Thumb opposition angle in degrees (default: 30.0)
- `--finger-threshold FINGER_THRESHOLD`: Finger detection distance threshold in mm (default: 100.0)
- `--thumb-threshold THUMB_THRESHOLD`: Thumb detection distance threshold in mm (default: 50.0)
- `--debug`: Enable debug logging

#### Design Features

1. **Robust against sensor errors**:
   - Uses any working finger sensor to trigger thumb opposition
   - Works even if some sensors are positioned incorrectly
   - Filters out sensor glitches with debouncing and hysteresis

2. **I2C Bus Contention Handling**:
   - Separates sensor reading and motor control phases
   - Implements a dedicated motion phase where sensors are ignored
   - Batches motor commands to minimize I2C bus activity

3. **Simple State Machine**:
   - IDLE: All fingers open, waiting for initial detection
   - THUMB_OPPOSING: Thumb positioned, waiting for thumb proximity
   - GRIP_CLOSING: All fingers closed to specified angle

### 2. Experiment Controller

`run_experiment.py` implements an experimental controller that makes fingers twitch instead of fully closing when objects are detected.

#### Usage

```bash
python3 -m prosthetic_control_system.controller.without_imu.run_experiment [options]
```

Options:
- `--simulate`: Use simulated hardware
- `--rate RATE`: Control loop rate in Hz (default: 20)
- `--port PORT`: Serial port for Ability Hand (default: auto-detect)
- `--proximity-rate PROXIMITY_RATE`: Proximity sensor sampling rate in Hz (default: 5)
- `--debug`: Enable debug logging