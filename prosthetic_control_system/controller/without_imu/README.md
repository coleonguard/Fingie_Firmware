# Prosthetic Control System Controllers

This directory contains controllers for the prosthetic control system that do not rely on IMU data.

## Controllers

### 1. Safe Thumb Opposition Controller

`safe_thumb_opposition_controller.py` implements a robust control approach that:

1. Rotates the thumb to opposition position when ANY finger detects an object
2. Closes the other fingers when the thumb detects an object
3. Handles I2C bus contention by separating sensor reading and motor control phases
4. Ensures thumb never collides with other fingers through careful movement sequencing

#### Usage

```bash
python3 -m prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller [options]
```

Options:
- `--simulate`: Use simulated hardware
- `--rate RATE`: Control loop rate in Hz (default: 20)
- `--port PORT`: Serial port for Ability Hand (default: auto-detect)
- `--proximity-rate PROXIMITY_RATE`: Proximity sensor sampling rate in Hz (default: 5)
- `--finger-close FINGER_CLOSE`: Finger closure angle in degrees (default: 30.0)
- `--thumb-oppose THUMB_OPPOSE`: Thumb opposition angle in degrees (default: 100.0, max rotation)
- `--finger-threshold FINGER_THRESHOLD`: Finger detection distance threshold in mm (default: 100.0)
- `--thumb-threshold THUMB_THRESHOLD`: Thumb detection distance threshold in mm (default: 50.0)
- `--debug`: Enable debug logging

#### Design Features

1. **Robust against sensor errors**:
   - Uses any working finger sensor to trigger thumb opposition
   - Works even if some sensors are positioned incorrectly
   - Double debouncing mechanism:
     - Requires two consecutive non-100mm readings to consider a detection valid
     - Further requires 3 consecutive readings below threshold for state changes
   - Prevents false triggers from single erroneous readings
   - Uses hysteresis to prevent oscillation at threshold boundaries

2. **I2C Bus Contention Handling**:
   - Separates sensor reading and motor control phases
   - Implements a dedicated motion phase where sensors are ignored
   - Batches motor commands to minimize I2C bus activity

3. **Simple State Machine**:
   - IDLE: All fingers open, waiting for initial detection
   - THUMB_OPPOSING: Thumb positioned, waiting for thumb proximity
   - GRIP_CLOSING: All fingers closed to specified angle

4. **Thumb Safety Features**:
   - Full 100° rotation of thumb for maximum opposition
   - Proper control of both thumb joints (ThumbRotate for opposition, Thumb for flexion)
   - Sequential movement to prevent thumb collisions with other fingers
   - Thumb always positions first before other fingers move
   - Non-thumb fingers always open first before thumb returns
   - Verification that thumb is in position before allowing finger closure

### 2. Wiggle Approach Controller

`wiggle_approach_controller.py` extends the Safe Thumb Opposition Controller by adding wiggle motion during the approach phase for enhanced tactile feedback.

#### Usage

```bash
python3 -m prosthetic_control_system.controller.without_imu.wiggle_approach_controller [options]
```

Options:
- All options from the Safe Thumb Opposition Controller, plus:
- `--wiggle-amplitude AMPLITUDE`: Maximum amplitude of wiggle motion in degrees (default: 5.0)
- `--wiggle-frequency FREQUENCY`: Frequency of wiggle motion in Hz (default: 0.5)

#### Additional Features

1. **Enhanced Tactile Feedback**:
   - Small, Gaussian-distributed random finger movements during approach
   - Randomly selected subset of fingers move at each update
   - Partially correlated movement between fingers (60% correlation)
   - Natural micro-movements that mimic human hand tremor patterns

2. **I2C Bus Handling with Wiggle**:
   - Extends the base controller's I2C bus contention handling
   - Manages the additional motor commands required for wiggling
   - Balances sensor reading and motor control phases

3. **Maintains Safety**:
   - Preserves all thumb safety features from the base controller
   - Ensures wiggle motion never interferes with safe thumb positioning

### 3. Experiment Controller

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