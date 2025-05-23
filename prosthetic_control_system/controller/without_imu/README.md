# Prosthetic Hand Controller Documentation

This document provides a comprehensive guide to the prosthetic hand controllers developed in this project. It includes detailed explanations of each controller's functionality, implementation details, and guidelines for extension.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Core Design Patterns](#core-design-patterns)
3. [Controller Implementations](#controller-implementations)
   - [Simple Phase Toggle Controller](#simple-phase-toggle-controller)
   - [Random Wiggle Controller](#random-wiggle-controller)
   - [Continuous Random Wiggle Controller](#continuous-random-wiggle-controller)
   - [Proximity Aware Wiggle Controller](#proximity-aware-wiggle-controller)
   - [Idle Wiggle Grasp Controller](#idle-wiggle-grasp-controller)
4. [Key Concepts](#key-concepts)
   - [Phase-Based Control](#phase-based-control)
   - [State Machine Implementation](#state-machine-implementation)
   - [Proximity Sensing and Debouncing](#proximity-sensing-and-debouncing)
   - [Random Movement Generation](#random-movement-generation)
5. [Parameter Reference](#parameter-reference)
6. [Extension Guide](#extension-guide)
7. [Troubleshooting](#troubleshooting)

## Architecture Overview

The prosthetic hand control system is built on a phase-based architecture that separates sensor reading operations from motor movement operations. This design addresses the fundamental hardware constraint of I2C bus contention between proximity sensors and the hand's motor controller.

The system follows these core principles:

1. **Strict Phase Separation**: All controllers alternate between sensor reading and movement phases to prevent I2C bus conflicts.
2. **State Machine Pattern**: Controllers use a state machine to manage different behavioral states.
3. **Visualization Integration**: Real-time feedback is provided through terminal-based visualization.
4. **Safety-First Design**: Multiple safety mechanisms prevent sudden movements and ensure smooth operation.

## Core Design Patterns

### Phase Toggle Pattern

All controllers implement a phase toggle pattern that alternates between:

- **Sensor Phase**: Reading proximity sensors and other input data
- **Movement Phase**: Sending motor commands to the hand

This pattern is implemented through the `_toggle_phase()` method that switches phases at regular intervals, with timing that can be fixed or randomized.

### State Machine Pattern

Controllers implement a state machine to manage different behavioral modes:

- **IDLE**: The default state where the hand is open or performing idle movements
- **THUMB_OPPOSING**: Positioning the thumb in opposition to prepare for grasping
- **GRASP_FORMING**: Closing the fingers around a detected object

State transitions are triggered by sensor events or command inputs, with each state having specific behavior rules and allowable transitions.

### Visualization Pattern

Controllers provide real-time feedback using terminal-based visualization:

- Position bars showing finger positions
- Direction indicators showing movement direction
- Proximity bars showing sensor readings
- State and phase indicators for system status

## Controller Implementations

### Simple Phase Toggle Controller

**File: simple_phase_toggle_test.py**

This controller demonstrates the basic phase toggle pattern with fixed timing. It provides a clean separation between sensor reading and movement phases without implementing complex movement patterns.

**Key Features:**
- Basic phase toggling between sensor reading and movement
- Fixed-duration phases
- Simple finger movement pattern (open/close)
- Terminal-based visualization of phase changes

**Implementation Details:**
- Phase duration: Fixed at 1.0 seconds for both phases
- Movement pattern: All fingers move in unison
- No state machine implementation (single state operation)

### Random Wiggle Controller

**File: random_wiggle_phase_test.py**

This controller builds upon the phase toggle pattern and adds randomized finger selection and movement during each phase.

**Key Features:**
- Random selection of fingers to move during each movement phase
- Variable movement durations using uniform random distribution
- Simple directional movement (always toward a fixed target)
- Basic visualization of finger positions

**Implementation Details:**
- Finger selection: Random subset of fingers selected for each movement phase
- Phase duration: Random between 0.5-2.0 seconds
- Movement amplitude: Fixed target positions for wiggling
- Movement direction: Single direction per phase

### Continuous Random Wiggle Controller

**File: continuous_random_wiggle_test.py**

This controller implements continuous random finger movements without returning to a neutral position between phases. It adds smart direction reversal to keep fingers within angle constraints.

**Key Features:**
- Continuous random finger movements with direction tracking
- Intelligent direction reversal based on position limits
- Variable movement amplitude using uniform random distribution
- Enhanced visualization showing movement direction

**Implementation Details:**
- Finger movement: Continuous with smart direction changes
- Movement amplitude: Random between 0-10 degrees per movement
- Position constraints: Fingers limited to 30% of maximum angle
- Direction management: Per-finger direction tracking

**Key Methods:**
- `_update_wiggle_targets()`: Generates random movement targets
- `_reverse_direction_if_needed()`: Ensures fingers stay within constraints

### Proximity Aware Wiggle Controller

**File: proximity_aware_wiggle_test.py**

This controller adds proximity sensor reading capabilities to the continuous random wiggle controller, providing awareness of nearby objects without changing behavior based on detection.

**Key Features:**
- Proximity sensor integration during sensor phase
- Enhanced visualization showing both finger positions and proximity readings
- Gaussian distribution for random timing
- All continuous wiggle features

**Implementation Details:**
- Sensor reading: All proximity sensors read during sensor phase
- Visualization: Added proximity sensor bars
- Phase duration: Gaussian random distribution for more natural timing
- No behavior change based on proximity (awareness only)

**Key Methods:**
- `_read_proximity_sensors()`: Reads and processes proximity sensor data
- `_visualize_proximity()`: Displays proximity readings

### Idle Wiggle Grasp Controller

**File: idle_wiggle_grasp_controller.py**

This is the most advanced controller, implementing a complete state machine with proximity-triggered grasping behavior. It combines all features from previous controllers and adds state-based behavior changes.

**Key Features:**
- Full state machine: IDLE, THUMB_OPPOSING, GRASP_FORMING
- Continuous random wiggling during IDLE state
- Proximity-triggered state transitions
- Multi-layer proximity debouncing
- Hysteresis for detection and release
- Sequential thumb opposition and finger grasping

**Implementation Details:**
- Idle behavior: Random finger wiggling when no object detected
- Detection mechanism: Multi-layer debouncing with hysteresis
- Thumb opposition: Positions thumb in opposition before grasping
- Grasping sequence: Progressive finger closure around detected object
- Release behavior: Returns to IDLE state when object removed

**State Machine:**
1. **IDLE**: Hand performs random finger wiggling
   - Transition to THUMB_OPPOSING when object consistently detected
   
2. **THUMB_OPPOSING**: Thumb moves to opposition position
   - Automatic transition to GRASP_FORMING when thumb positioned
   - Return to IDLE if object lost during positioning
   
3. **GRASP_FORMING**: Fingers close around the detected object
   - Return to IDLE if object removed (with hysteresis)

**Key Methods:**
- `_handle_idle_state()`: Manages wiggling behavior
- `_handle_thumb_opposing_state()`: Positions thumb for grasping
- `_handle_grasp_forming_state()`: Controls grasping movement
- `_process_proximity_readings()`: Implements multi-layer debouncing
- `_select_random_fingers()`: Randomizes finger selection for each movement phase

## Key Concepts

### Phase-Based Control

Phase-based control is fundamental to all controllers and addresses the hardware constraint of I2C bus contention between proximity sensors and motor control.

**Implementation Details:**
- Phases alternate between sensor reading and movement
- Each phase has a distinct duration (fixed or random)
- Phase transitions are managed by the `_toggle_phase()` method
- Timing is controlled by `next_phase_time` which sets when the next transition occurs

**Example Implementation:**
```python
def _toggle_phase(self):
    """Toggle between sensor reading and movement phases with random durations"""
    self.phase_count += 1
    
    if self.current_phase == "SENSOR_PHASE":
        # Switch to movement phase
        self.current_phase = "MOVEMENT_PHASE"
        
        # Generate random phase duration
        phase_duration = self._generate_phase_duration()
        self.next_phase_time = time.time() + phase_duration
        
        logger.info(f"🔄 SWITCHING TO MOVEMENT PHASE - duration: {phase_duration:.2f}s")
        
        # Select new random fingers to wiggle for this movement phase
        if self.state == "IDLE":
            self._select_random_fingers()
    else:
        # Switch to sensor phase
        self.current_phase = "SENSOR_PHASE"
        
        # Generate random phase duration
        phase_duration = self._generate_phase_duration()
        self.next_phase_time = time.time() + phase_duration
        
        logger.info(f"🔄 SWITCHING TO SENSOR PHASE - duration: {phase_duration:.2f}s")
```

### State Machine Implementation

The state machine pattern is fully implemented in the Idle Wiggle Grasp Controller, providing distinct behavioral modes based on environmental conditions.

**States:**
1. **IDLE**: Default state with continuous random finger wiggling
2. **THUMB_OPPOSING**: Transitional state for thumb positioning
3. **GRASP_FORMING**: Active grasping state when object detected

**Transition Logic:**
- IDLE → THUMB_OPPOSING: Triggered by consistent object detection
- THUMB_OPPOSING → GRASP_FORMING: Automatic after thumb positioning
- THUMB_OPPOSING → IDLE: If object lost during positioning
- GRASP_FORMING → IDLE: If object removed (with hysteresis)

**Example Implementation:**
```python
def _update_state(self):
    """Update the controller state based on proximity readings"""
    if self.state == "IDLE":
        # Check if we should transition to THUMB_OPPOSING
        if self.object_detected:
            self.state = "THUMB_OPPOSING"
            logger.info("🔄 State change: IDLE → THUMB_OPPOSING")
            self._start_thumb_opposition()
    
    elif self.state == "THUMB_OPPOSING":
        # Check if thumb opposition is complete
        if self.thumb_opposition_complete:
            self.state = "GRASP_FORMING"
            logger.info("🔄 State change: THUMB_OPPOSING → GRASP_FORMING")
            self._start_grasp_formation()
        
        # Return to IDLE if object is lost during thumb opposition
        elif not self.object_detected:
            self.state = "IDLE"
            logger.info("🔄 State change: THUMB_OPPOSING → IDLE (object lost)")
    
    elif self.state == "GRASP_FORMING":
        # Return to IDLE if object is removed
        if not self.object_detected:
            self.state = "IDLE"
            logger.info("🔄 State change: GRASP_FORMING → IDLE (object removed)")
```

### Proximity Sensing and Debouncing

Proximity sensing is critical for object detection and uses multi-layer debouncing to ensure reliable detection.

**Debouncing Layers:**
1. **Initial Filtering**: Reject first non-100mm readings after startup
2. **Consecutive Reading Filter**: Reject identical consecutive readings
3. **Valid Reading Counter**: Require multiple consecutive valid readings
4. **Hysteresis**: Different thresholds for detection and release

**Example Implementation:**
```python
def _process_proximity_readings(self, proximity_values):
    """Process proximity readings with multi-layer debouncing"""
    # Initialize detection flags for each finger
    finger_detections = {finger: False for finger in self.fingers}
    
    # Process each finger's proximity reading
    for finger, reading in proximity_values.items():
        # Skip invalid readings
        if reading <= 0 or reading >= 100:
            continue
            
        # Apply detection threshold with hysteresis
        if not self.finger_detected[finger] and reading < self.DETECTION_THRESHOLD:
            # Increment consecutive detection count
            self.consecutive_detections[finger] += 1
            
            # Mark as detected after sufficient consecutive detections
            if self.consecutive_detections[finger] >= self.REQUIRED_CONSECUTIVE_DETECTIONS:
                finger_detections[finger] = True
                
        elif self.finger_detected[finger] and reading > self.RELEASE_THRESHOLD:
            # Increment consecutive release count
            self.consecutive_releases[finger] += 1
            
            # Mark as released after sufficient consecutive releases
            if self.consecutive_releases[finger] >= self.REQUIRED_CONSECUTIVE_RELEASES:
                finger_detections[finger] = False
                
        else:
            # Reset counters if reading is inconsistent
            self.consecutive_detections[finger] = 0
            self.consecutive_releases[finger] = 0
    
    # Update finger detection status
    self.finger_detected = finger_detections
    
    # Determine overall object detection status
    self.object_detected = any(finger_detections.values())
```

### Random Movement Generation

Random movement patterns are used to create natural-looking finger wiggling, with several key mechanisms:

**Random Finger Selection:**
- Random subset of fingers selected for each movement phase
- Implemented in `_select_random_fingers()` method
- Different selection strategies based on controller needs

**Random Movement Amplitude:**
- Uniform random distribution for movement amplitude
- Typically 0-10 degrees of movement per phase
- Constrained to keep fingers within 30% of maximum angle

**Random Phase Duration:**
- Either uniform or Gaussian distribution for timing
- Typically 0.5-2.0 seconds per phase
- Implemented in `_generate_phase_duration()` method

**Direction Management:**
- Per-finger direction tracking
- Smart reversal when approaching limits
- Prevents exceeding angle constraints

**Example Implementation:**
```python
def _select_random_fingers(self):
    """Select a random subset of fingers to wiggle during this movement phase"""
    # Randomly select between 1-3 fingers to wiggle
    num_fingers = random.randint(1, min(3, len(self.fingers)))
    self.active_fingers = random.sample(self.fingers, num_fingers)
    
    # Initialize or update movement directions for the active fingers
    for finger in self.active_fingers:
        if finger not in self.finger_directions or random.random() < 0.3:
            # 30% chance to switch direction even if finger was already active
            self.finger_directions[finger] = random.choice([-1, 1])
    
    logger.info(f"Selected fingers for wiggling: {self.active_fingers}")

def _update_wiggle_targets(self):
    """Update target positions for wiggling fingers"""
    for finger in self.active_fingers:
        # Get current position
        current_pos = self.hand.get_finger_position(finger)
        
        # Generate random movement amount (0-10 degrees)
        movement_amount = random.uniform(0, 10)
        
        # Apply direction and convert to normalized units
        movement = (movement_amount / 100.0) * self.finger_directions[finger]
        
        # Calculate new target position
        new_target = current_pos + movement
        
        # Ensure position stays within bounds
        new_target = self._constrain_position(finger, new_target)
        
        # Reverse direction if necessary
        self._reverse_direction_if_needed(finger, current_pos, new_target)
        
        # Set the new target
        self.finger_targets[finger] = new_target
```

## Parameter Reference

### Common Parameters

These parameters are used across multiple controllers:

| Parameter | Description | Typical Value | Used In |
|-----------|-------------|---------------|---------|
| `SENSOR_PHASE_DURATION` | Duration of sensor reading phase | 1.0s | Simple Phase Toggle |
| `MOVEMENT_PHASE_DURATION` | Duration of movement phase | 1.0s | Simple Phase Toggle |
| `MIN_PHASE_DURATION` | Minimum random phase duration | 0.5s | All random controllers |
| `MAX_PHASE_DURATION` | Maximum random phase duration | 2.0s | All random controllers |
| `MAX_WIGGLE_PERCENTAGE` | Maximum finger angle as percentage | 30% | All wiggle controllers |
| `DETECTION_THRESHOLD` | Proximity threshold for object detection | 50mm | Proximity-aware controllers |
| `RELEASE_THRESHOLD` | Proximity threshold for object release | 70mm | Proximity-aware controllers |
| `REQUIRED_CONSECUTIVE_DETECTIONS` | Count of readings needed for detection | 3 | Idle Wiggle Grasp |
| `REQUIRED_CONSECUTIVE_RELEASES` | Count of readings needed for release | 5 | Idle Wiggle Grasp |

### Idle Wiggle Grasp Parameters

These parameters are specific to the Idle Wiggle Grasp Controller:

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `THUMB_OPPOSITION_POSITION` | Thumb position for opposition | 0.6 (60%) |
| `GRASP_POSITION` | Finger position for grasping | 0.8 (80%) |
| `GRASP_SPEED` | Speed for grasping movement | 0.3 (30%) |
| `OPPOSITION_SPEED` | Speed for thumb opposition | 0.2 (20%) |
| `WIGGLE_SPEED` | Speed for wiggle movements | 0.15 (15%) |
| `MIN_FINGERS_TO_WIGGLE` | Minimum fingers to wiggle | 1 |
| `MAX_FINGERS_TO_WIGGLE` | Maximum fingers to wiggle | 3 |
| `DETECTION_HYSTERESIS` | Difference between detection and release thresholds | 20mm |

## Extension Guide

### Adding a New Controller

To create a new controller based on the existing architecture:

1. **Choose a Base Controller**: Start with the controller that most closely matches your needs
2. **Implement Phase Toggling**: Always include the phase toggle pattern for I2C safety
3. **Define State Machine**: If needed, implement a state machine with clear states and transitions
4. **Add Visualization**: Include real-time visualization for debugging and feedback
5. **Test Incrementally**: Test each component individually before full integration

### Modifying Existing Controllers

When modifying existing controllers, follow these guidelines:

1. **Preserve Phase Separation**: Never remove or bypass the phase toggle mechanism
2. **Maintain Safety Checks**: Keep movement constraints and safety checks intact
3. **Document Changes**: Add comments explaining modifications
4. **Test Thoroughly**: Verify that modifications work with actual hardware

### Common Extension Points

These areas are designed for extension:

1. **Movement Patterns**: Modify the `_update_wiggle_targets()` method
2. **State Transitions**: Extend the `_update_state()` method
3. **Sensor Processing**: Enhance the `_process_proximity_readings()` method
4. **Visualization**: Extend the visualization methods for new data types

## Troubleshooting

### Common Issues

1. **Erratic Finger Movement**
   - **Cause**: I2C bus contention or phase timing issues
   - **Solution**: Ensure phase durations are sufficient; increase if necessary

2. **False Detections**
   - **Cause**: Insufficient debouncing or environmental factors
   - **Solution**: Increase `REQUIRED_CONSECUTIVE_DETECTIONS` or adjust thresholds

3. **Unresponsive Fingers**
   - **Cause**: Rate limiting may be too restrictive
   - **Solution**: Check and adjust rate limits in the hand interface

4. **Visualization Lag**
   - **Cause**: Excessive logging or visualization updates
   - **Solution**: Reduce visualization frequency or simplify display

### Debugging Tips

1. Enable DEBUG level logging to see detailed operation
2. Add temporary print statements for specific values
3. Use the visualization to track finger positions and state changes
4. Run with the `--verbose` flag for additional diagnostic information

## Conclusion

This documentation provides a comprehensive guide to the prosthetic hand controllers developed in this project. By understanding the core design patterns, implementation details, and extension points, developers can effectively maintain and extend these controllers for future applications.

The modular, phase-based architecture ensures safe and reliable operation while providing a flexible foundation for advanced behavior development. The progression from simple phase toggling to complex state machines with proximity-triggered behavior demonstrates the evolutionary development approach used in this project.