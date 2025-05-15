# Velocity-Based Proximity Control for Prosthetic Hand

This implementation explores using velocity control instead of position control for prosthetic hand movements based on proximity sensor inputs. The goal is to reduce jitteriness and create more natural, fluid finger movements.

## Theoretical Advantages of Velocity Control

The standard position-based controller has several limitations that may contribute to jerky motion:

1. **Discrete Position Jumps**: When sensor readings change, position targets make sudden jumps
2. **Motion Discontinuity**: Each position update creates a new motion trajectory without continuity
3. **Abrupt Stops**: When sensor readings are briefly lost, the finger freezes at its current position
4. **No Motion Inertia**: Position control lacks natural motion dynamics like acceleration and momentum

A velocity-based approach addresses these limitations by:

1. **Continuous Motion**: Converting proximity readings to velocity commands creates smoother transitions
2. **Motion Inertia**: Fingers continue moving with decreasing velocity when sensor readings stop
3. **Momentum Simulation**: Gradual acceleration and deceleration create more natural finger movement
4. **Dynamic Response**: Velocity changes feel more responsive than position jumps

## Implementation Details

### Key Components

1. **Velocity Command Integration**: Instead of directly commanding finger positions, we send velocity commands to the Ability Hand
2. **Internal Position Estimation**: The controller tracks estimated finger positions to enforce limits and prevent over-extension
3. **Velocity Damping System**: Implements smooth velocity transitions with configurable damping factors
4. **Safety Bounds**: Software limits prevent over-extension or over-flexion of fingers
5. **Periodic Position Recalibration**: Occasional position commands correct any drift in estimated positions

### Architecture Changes

This controller differs from the position-based implementation in these key ways:

1. **Control Output**: Uses `set_velocity()` instead of `set_position()` to command the hand
2. **State Machine Outputs**: Finger states map to velocity targets instead of position targets
3. **Motion Model**: Implements damping and inertia effects on velocities
4. **Safety System**: Adds velocity limiting and position boundary enforcement
5. **Position Tracking**: Maintains internal position estimates based on applied velocities

### Parameters

The velocity controller has several tunable parameters:

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `VELOCITY_SCALE` | Scaling factor for velocity commands | 30-100 deg/sec |
| `VELOCITY_DAMPING` | Smoothing factor for velocity changes (higher = smoother) | 0.7-0.95 |
| `MAX_CLOSING_VELOCITY` | Maximum closing velocity | 50-100 deg/sec |
| `MAX_OPENING_VELOCITY` | Maximum opening velocity | 50-150 deg/sec |
| `POSITION_RECALIBRATION_INTERVAL` | How often to recalibrate position (seconds) | 5-30 sec |
| `IDLE_RETURN_FACTOR` | How quickly fingers return to open position when idle | 0.2-0.5 |

## Warning About API Stability

**IMPORTANT:** The Ability Hand API documentation contains an explicit warning about potential instability in velocity control mode:

```python
"WARNING VELOCITY TARGETS UNSTABLE AT THE MOMENT AND MAY CAUSE THUMB OSCILLATIONS, SUGGEST USING POSITION INSTEAD"
```

This implementation acknowledges this risk but explores velocity control for potential benefits in motion smoothness. Use with caution and monitor for thumb oscillations or other instability.

## Usage

Run the velocity controller with:

```bash
# With connected hardware
python -m prosthetic_control_system.controller.without_imu.velocity_controller.run_velocity_controller

# In simulation mode (no hardware)
python -m prosthetic_control_system.controller.without_imu.velocity_controller.run_velocity_controller --simulate

# With specific serial port
python -m prosthetic_control_system.controller.without_imu.velocity_controller.run_velocity_controller --port /dev/ttyUSB0
```

## Tuning Guidelines

For optimal performance and to reduce jitteriness:

1. Start with a higher `VELOCITY_DAMPING` value (0.9) for smoother transitions
2. Adjust `VELOCITY_SCALE` based on desired responsiveness
3. If fingers oscillate or show instability, reduce `MAX_CLOSING_VELOCITY` and `MAX_OPENING_VELOCITY`
4. For fingers that drift from their actual positions, reduce the `POSITION_RECALIBRATION_INTERVAL`
5. If the hand doesn't return to open position quickly enough when idle, increase `IDLE_RETURN_FACTOR`

## Comparison with Position Control

| Aspect | Position Control | Velocity Control |
|--------|------------------|------------------|
| Stability | More stable according to API | Potential for oscillations |
| Smoothness | Can be jerky with sensor fluctuations | Potentially smoother with dampening |
| Response to sensor dropouts | Immediate stop | Continues with damped motion |
| Position accuracy | Directly controls position | May drift between recalibrations |
| Implementation complexity | Simpler | Requires more state tracking |
| API recommendation | Recommended | Not recommended (but worth exploring) |

## Implementation Notes

This implementation is experimental and designed to compare with the position-based approach. It may require adjustment for specific hardware configurations and user preferences.