# Prosthetic Hand Controllers: Technical Documentation

This document provides detailed technical information about the prosthetic hand controllers in this directory, their operation, design considerations, and development notes.

## Controller Overview

This directory contains controllers for the prosthetic hand system that operate without requiring IMU data. Each controller has a specific focus:

1. **run_experiment.py**: Experimental controller with finger twitching for testing
2. **safe_thumb_opposition_controller.py**: Primary controller with reliable thumb opposition
3. **wiggle_approach_controller.py**: Enhanced controller with tactile feedback through finger wiggling

## File Descriptions

### `config.py`

Configuration file with default settings, sensor mappings, and thresholds. Contains:
- Default control parameters
- Multiplexer addresses and sensor configurations
- Finger-to-motor mappings
- Fallback sensor mappings for reliability
- Default thresholds and timing settings

### `run_experiment.py`

Experimental controller designed to make fingers twitch rather than fully close when objects are detected:
- Creates small (5°) closing movements followed by reopening
- Implements basic I2C bus contention management
- Uses state-based approach for managing finger movements
- Maintains independent finger control based on sensor readings

### `safe_thumb_opposition_controller.py`

Primary controller implementing reliable thumb opposition grasp control:
- Uses full 100° rotation of thumb for proper opposition
- Properly controls both thumb joints (ThumbRotate for opposition, Thumb for flexion)
- Implements careful movement sequencing for safety
- Contains robust sensor validation and debouncing
- Manages I2C bus contention through motion phases

### `wiggle_approach_controller.py`

Enhanced controller that extends the safe controller with finger wiggling:
- Inherits all safety features from safe_thumb_opposition_controller
- Adds randomized Gaussian-distributed finger movements during approach phase
- Implements random finger subset selection for more natural motion
- Contains partially correlated finger movements

## Core Design Elements

### State Machine

All controllers use a three-state approach:
1. **IDLE**: All fingers open, waiting for object detection
2. **THUMB_OPPOSING**: Thumb positioned in opposition, waiting for closer approach
3. **GRIP_CLOSING**: All fingers closed to specified angle

### Thumb Safety Features

Careful movement sequencing prevents thumb-finger collisions:
1. When positioning thumb: Thumb moves first, then other fingers
2. When closing grip: Verify thumb is properly positioned before closing fingers
3. When opening: Non-thumb fingers open first, then thumb
4. When stopping: Proper sequence for shutdown

### Sensor Reading Validation

Two-layer debouncing mechanism for reliable sensor readings:
1. **Layer 1**: Validates raw readings
   - Requires two consecutive non-identical readings < 100mm
   - Rejects identical non-100mm readings as likely glitches
   - Prevents sporadic false triggers

2. **Layer 2**: Validates state transitions
   - Requires 3 consecutive valid readings below threshold
   - Adds hysteresis to prevent oscillation at threshold boundaries
   - Ensures intentional transitions

### I2C Bus Contention Management

Carefully manages shared I2C bus between sensors and motors:
1. Separates operation into discrete sensing and motion phases
2. Uses minimal motion phase duration (0.4s) to maximize responsiveness
3. Applies all motor commands in a single motion phase
4. Parameterized movement timing for easy tuning

## Key Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| control_rate | Control loop frequency | 20 Hz |
| finger_close_angle | Angle for finger closure | 30° |
| thumb_opposition_angle | Angle for thumb opposition | 100° (max) |
| finger_threshold | Detection threshold for fingers | 100 mm |
| thumb_threshold | Detection threshold for thumb | 50 mm |
| hysteresis | Distance buffer for state transitions | 20 mm |
| motion_phase_duration | Time spent in motion phase | 0.4 s |
| thumb_move_time | Time allowed for thumb movement | 0.2 s |
| finger_move_time | Time allowed for finger movement | 0.2 s |
| readings_required | Consecutive readings for state change | 3 |
| wiggle_amplitude | Maximum amplitude of wiggle motion | 5° |
| wiggle_mean_interval | Mean time between wiggles | 0.7 s |
| wiggle_std_dev | Standard deviation for wiggle timing | 0.2 s |
| wiggle_min_interval | Minimum time between wiggles | 0.2 s |

## Implementation Details

### Proximity Sensor Reading

1. Sensors are read at ~20 Hz (every 50ms)
2. Readings go through a validation process to filter glitches
3. The system uses the minimum distance from any working finger sensor

### Motor Control

1. Proper joint selection:
   - `ThumbRotate`: Controls thumb opposition (rotational joint)
   - `Thumb`: Controls thumb flexion (minimal during opposition)
   - Other fingers: Standard position control

2. Sequential movement for safety:
   - First position thumb, then other fingers
   - Wait for each movement to complete

### Wiggle Implementation (wiggle_approach_controller only)

1. Gaussian Random Timing:
   - Uses a normal distribution centered around 0.7 seconds for wiggle intervals
   - Standard deviation of 0.2 seconds creates natural timing variation
   - Ensures minimum interval of 0.2 seconds between wiggles
   - Creates unpredictable, human-like timing pattern

2. Random Finger Subset Selection:
   - Randomly selects which fingers to move at each wiggle
   - Number of fingers ranges from 1 to all available non-thumb fingers
   - Each wiggle uses a different combination of fingers

3. Gaussian Position Distribution:
   - Uses statistical normal distribution for finger positions
   - Standard deviation scaled by amplitude parameter
   - Creates natural variance in movement amplitudes

4. Partial Correlation:
   - 60% correlation between finger movements
   - Mimics natural coupling of human hand motion
   - Ensures movements appear coordinated but not identical

## Tuning Guidelines

### Sensor Debouncing

If experiencing false triggers:
1. Increase `readings_required` (default: 3)
2. Decrease control rate to get more stable readings

If experiencing missed detections:
1. Decrease `readings_required`
2. Lower finger_threshold and thumb_threshold values

### Motion Timing

If fingers collide:
1. Increase `thumb_move_time` and `finger_move_time`
2. Increase `motion_phase_duration` accordingly

If responsiveness is too slow:
1. Decrease `min_dwell_time`
2. Decrease `motion_phase_duration` (minimum safe value ~0.3s)

### Grasp Parameters

For stronger grip:
1. Increase `finger_close_angle` (max ~90°)

For more sensitivity:
1. Increase `finger_threshold` and `thumb_threshold`

## Development Notes

### Current Limitations

1. **I2C Bus Contention**: The current design blocks sensor reading during motion phases
2. **Sequential Processing**: No parallel execution of sensing and motor control
3. **Limited Sensor Diagnosis**: Basic error reporting for sensor failures

### Future Improvements

1. **Asynchronous Operation**:
   - Implement true parallel sensor reading and motor control
   - Use separate thread for continuous sensor monitoring

2. **Adaptive Motion Timing**:
   - Dynamically adjust motion phase duration based on observed motor response
   - Implement non-blocking movement with progress tracking

3. **Enhanced Tactile Feedback**:
   - Improve wiggle patterns based on user testing
   - Implement variable wiggle parameters based on detected object properties

4. **Predictive Sensing**:
   - Implement trajectory prediction for faster response to approaching objects
   - Use sensor history to anticipate movement patterns

5. **Sensor Fusion**:
   - Incorporate tactile feedback once available
   - Add weight/pressure sensing to grip control

## Troubleshooting

### Common Issues

1. **Unresponsive fingers**:
   - Check sensor connections and multiplexer addresses
   - Verify finger mapping in config.py
   - Increase control_rate for more frequent updates

2. **False triggers**:
   - Enable debug logging to identify problematic sensors
   - Increase debouncing parameters
   - Check for electromagnetic interference sources

3. **Thumb collision**:
   - Increase thumb_move_time to ensure proper positioning
   - Verify ThumbRotate joint is functional
   - Check max_rotation value (should be 100°)

4. **Slow response**:
   - Decrease motion_phase_duration (minimum ~0.3s)
   - Increase control_rate
   - Decrease readings_required (minimum 2)

### Logging

Enable debug logging with the `--debug` flag to get detailed operation logs:

```bash
python3 -m prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller --debug
```

## Usage Examples

### Safe Thumb Opposition Controller

```bash
# Basic usage with simulation
python3 -m prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller --simulate

# With real hardware and custom parameters
python3 -m prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller --finger-close 40 --thumb-threshold 60
```

### Wiggle Approach Controller

```bash
# Basic usage with simulation
python3 -m prosthetic_control_system.controller.without_imu.wiggle_approach_controller --simulate

# With custom wiggle parameters
python3 -m prosthetic_control_system.controller.without_imu.wiggle_approach_controller --wiggle-amplitude 3 --wiggle-frequency 0.7
```

### Experimental Controller

```bash
# Basic usage with simulation
python3 -m prosthetic_control_system.controller.without_imu.run_experiment --simulate
```