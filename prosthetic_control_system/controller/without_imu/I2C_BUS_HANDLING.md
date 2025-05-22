# I2C Bus Contention Management

This document explains the approach used to handle I2C bus contention between proximity sensors and motor controllers in the prosthetic hand system.

## The Challenge

The proximity sensors and motor controllers in the prosthetic hand share the same I2C bus, which creates potential contention when trying to:
1. Read from sensors to detect objects
2. Send motor commands to move fingers
3. Get motor position/current feedback

Attempting to do these operations simultaneously can lead to:
- Bus errors
- Incomplete commands
- Missed readings
- Communication failures

## Current Implementation

### Time Division Approach

The controllers use a time-division approach with discrete operational phases:

1. **Sensor Reading Phase**:
   - Read all proximity sensors
   - Update state machine based on readings
   - Process debouncing logic
   - No motor commands are sent during this phase

2. **Motion Phase**:
   - Enter motion phase when state requires finger movement
   - Temporarily pause sensor reading for a fixed duration
   - Send all motor commands in a planned sequence
   - Return to sensor reading after the motion phase completes

### Key Parameters

```python
# From safe_thumb_opposition_controller.py
self.thumb_move_time = 0.2     # seconds for thumb to move to position
self.finger_move_time = 0.2    # seconds for fingers to move to position
self.motion_phase_duration = 0.4  # Total motion phase duration
```

### Control Flow

The main control loop implements this logic:

```python
def _control_loop(self):
    while self.running:
        # Check if it's time for the next control update
        if start_time >= next_update_time:
            # If we're in motion phase, skip sensor reading
            if self.in_motion_phase:
                if time.time() - self.motion_start_time > self.motion_phase_duration:
                    self.in_motion_phase = False
            else:
                # Read sensors and update state
                self._read_sensors_and_update_state()
                
                # If state requires motion, enter motion phase and move fingers
                if self._should_move():
                    self._enter_motion_phase()
                    self._move_fingers()
```

## Benefits and Drawbacks

### Benefits
- **Simplicity**: Clear separation of responsibilities
- **Reliability**: Prevents I2C conflicts by design
- **Safety**: Guarantees each operation completes without interruption
- **Determinism**: Behavior is highly predictable

### Drawbacks
- **Responsiveness**: Motion phase introduces a latency window
- **Lost Sensor Data**: Sensor changes during motion phases are missed
- **Fixed Duration**: Uses worst-case timing rather than adaptive

## Parameter Tuning

For optimal performance, the following parameters can be tuned:

1. **Motion Phase Duration** (`motion_phase_duration`):
   - Minimum necessary time for fingers to complete movement
   - Shorter = more responsive but might not complete movements
   - Longer = ensures movements complete but more latency

2. **Movement Times** (`thumb_move_time`, `finger_move_time`):
   - Time allocated for physical movement of motors
   - Should match actual motor performance
   - Too short = potential collisions
   - Too long = unnecessary delay

3. **Control Rate** (`control_rate`):
   - Frequency of the main control loop
   - Higher = more responsive but more CPU usage
   - Lower = more stable but less responsive

## Alternative Approaches

### 1. Non-Blocking Movement

An alternative would be to use a non-blocking approach where:
- Send motor commands and continue sensor reading
- Track motor progress without sleep/blocking
- Handle potential I2C bus errors with retries

```python
# Pseudocode for non-blocking approach
def _move_fingers_non_blocking(self):
    # Send commands to motors
    for finger, position in target_positions.items():
        self.motors.set_position_async(finger, position)
    
    # Continue with other operations, motors will move in background
```

Challenges: More complex error handling, requires motor interface that supports async operations.

### 2. Prioritized Scheduling

A priority-based scheduler could:
- Assign higher priority to critical operations
- Allow important sensor readings to interrupt motor operations
- Balance resources based on current state

```python
# Pseudocode for priority scheduling
def _schedule_operations(self):
    if critical_sensor_reading_needed():
        read_critical_sensors()
    elif motor_movement_in_progress():
        continue_motor_movement()
    else:
        read_all_sensors()
```

Challenges: Complex priority determination, risk of starvation for low-priority operations.

### 3. Split Bus Implementation

A hardware solution would be to:
- Use separate I2C buses for sensors and motors
- Eliminate contention at the hardware level
- Allow true parallel operation

Challenges: Requires hardware modification, additional microcontroller ports.

## Future Improvements

1. **Adaptive Motion Phase Duration**:
   - Measure actual motor movement time
   - Dynamically adjust motion phase duration
   - End motion phase as soon as movement completes

2. **Event-Based Interruption**:
   - Allow significant sensor events to interrupt motion phase
   - Emergency stop capability for safety

3. **Partial Sensor Reading**:
   - Continue reading only critical sensors during motion phase
   - Use reduced sensing frequency during motor operation

## Implementation Example

Here's a simplified example showing how to implement an adaptive motion phase:

```python
def _control_loop_adaptive(self):
    while self.running:
        if self.in_motion_phase:
            # Check if motors have reached target positions
            motors_settled = True
            for finger in self.motors.fingers:
                current = self.motors.get_position(finger)
                target = self.target_positions[finger]
                if abs(current - target) > 2.0:  # 2° tolerance
                    motors_settled = False
                    break
                    
            # End motion phase when motors settle or max time reached
            if motors_settled or (time.time() - self.motion_start_time > self.max_motion_duration):
                self.in_motion_phase = False
        else:
            # Normal sensor reading and state updates
            # ...
```

## Conclusion

The current time-division approach offers a good balance of reliability and simplicity, while future development could explore more responsive approaches like adaptive timing or true parallel processing for improved performance.

For most situations, the current implementation will function well, but tuning the motion phase duration is critical for optimizing the balance between safety and responsiveness.