# I2C Bus Contention Handling with Wiggle Motion

This document explains the approaches used in the controllers to handle I2C bus contention between proximity sensor readings and motor control, particularly focusing on the implementation of the wiggle motion.

## The I2C Contention Challenge

The prosthetic hand system faces a critical challenge: **the proximity sensors and motor controllers share the same I2C bus**, which can lead to communication conflicts when trying to read from sensors and write to motors simultaneously.

This issue becomes even more pronounced when implementing a wiggle motion during approach, as this requires frequent motor position updates while still maintaining awareness of proximity readings.

## Solution Approaches

### 1. Time Division Approach (Implemented)

The current implementation uses a time division approach with discrete phases:

1. **Sensor Reading Phase**:
   - Read all proximity sensors in a batch
   - Update the state machine based on these readings
   - No motor commands are sent during this phase

2. **Motion Phase**:
   - When motion is needed, enter a dedicated motion phase
   - Temporarily ignore sensors during this phase
   - Send all motor commands in sequence
   - Return to sensor reading after a fixed duration

Benefits:
- Simple implementation with clear separation of concerns
- Prevents I2C bus collisions
- Allows for complex motion patterns like sequenced finger movements

Drawbacks:
- Creates a trade-off between motion responsiveness and sensing responsiveness
- Might miss rapid changes in proximity during motion phases

### 2. Single-Device Focus Approach (Alternative)

An alternative approach would be to focus on one device at a time:

1. Read from a single proximity sensor
2. Update finger position if needed
3. Move to the next sensor
4. Repeat the cycle

Benefits:
- More responsive to individual sensor changes
- Can interleave sensor readings and motor commands

Drawbacks:
- More complex implementation
- Less predictable timing
- Difficult to coordinate multi-finger movements

### 3. Prioritized Scheduling Approach (Alternative)

Another approach would be to use a priority-based scheduler:

1. Assign higher priority to critical sensor readings
2. Assign lower priority to wiggle motion updates
3. Allow high-priority operations to interrupt low-priority ones
4. Ensure critical state transitions are never missed

Benefits:
- More flexible resource allocation
- Better balance between responsiveness and motion smoothness

Drawbacks:
- Significantly more complex implementation
- Requires careful tuning of priorities

## Implementation Details for Wiggle Motion

The current WiggleApproachController implements the wiggle motion while handling I2C bus contention through:

1. **Extended Motion Phase**:
   - The motion phase duration is increased to accommodate the additional finger movements
   - This ensures all wiggle position updates occur without sensor interruptions

2. **Efficient Motion Calculations**:
   - Wiggle positions are pre-calculated before sending any motor commands
   - This minimizes the time spent in the motion phase

3. **Time-Limited Updates**:
   - Wiggle updates are limited by `min_wiggle_interval` (0.1s)
   - This prevents excessive motor commands that would monopolize the I2C bus

4. **Phase-Based Motion**:
   - Each finger has a different phase offset
   - This creates a more natural, wave-like motion
   - It also distributes motor commands more evenly

## Future Improvements

Several improvements could make the wiggle motion more effective while maintaining I2C bus stability:

1. **Adaptive Motion Phase Duration**:
   - Dynamically adjust the motion phase duration based on proximity readings
   - Use shorter motion phases when objects are very close (more frequent sensing)
   - Use longer motion phases when objects are farther away (more fluid motion)

2. **Sensor-Specific Prioritization**:
   - Prioritize reading from sensors that most recently detected nearby objects
   - De-prioritize sensors that consistently show no objects nearby

3. **Predictive Motion Planning**:
   - Predict object movement trajectories based on past readings
   - Adjust wiggle patterns to better detect the predicted object path

4. **Hardware-Level I2C Improvements**:
   - Explore adding a second I2C bus to separate sensors and motors
   - Investigate faster I2C clock speeds for shorter transaction times
   - Consider interrupt-driven I2C instead of polling

## Conclusion

The current implementation strikes a balance between robust I2C bus handling and responsive finger control. By separating sensor reading and motor control into distinct phases, it ensures stable operation while still providing the enhanced tactile feedback of wiggling fingers during approach.

For most applications, this time-division approach will provide the best combination of reliability and functionality. However, for specialized applications requiring extremely responsive sensing or very complex motion patterns, one of the alternative approaches might be worth exploring.