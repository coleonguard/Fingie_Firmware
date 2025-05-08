# Hardware Integration Guide

This guide provides step-by-step instructions for testing, calibrating, and integrating hardware components for the prosthetic control system.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Hardware Testing Sequence](#hardware-testing-sequence)
3. [Individual Component Testing](#individual-component-testing)
   - [I2C Multiplexers](#i2c-multiplexers)
   - [VL6180X Proximity Sensors](#vl6180x-proximity-sensors)
   - [IMU](#imu)
   - [Ability Hand Motors](#ability-hand-motors)
4. [Calibration Procedures](#calibration-procedures)
   - [Proximity Sensor Calibration](#proximity-sensor-calibration)
   - [IMU Calibration](#imu-calibration)
5. [Integration Testing](#integration-testing)
6. [Data Collection and Analysis](#data-collection-and-analysis)
7. [Troubleshooting](#troubleshooting)

## Prerequisites

Before beginning hardware integration, ensure you have:

1. All hardware components:
   - I2C multiplexers (TCA9548A)
   - VL6180X proximity sensors
   - Microstrain IMU
   - Ability Hand prosthetic
   
2. Necessary dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Access to I2C bus and serial ports:
   ```bash
   # On Linux, ensure your user has access to I2C and serial devices
   sudo usermod -a -G dialout,i2c $USER
   # Log out and log back in for changes to take effect
   ```

## Hardware Testing Sequence

For systematic hardware integration, follow this testing sequence:

1. **Hardware Connectivity Tests**
   - Test I2C multiplexers
   - Test IMU connectivity
   - Test Ability Hand communication

2. **Component-Level Tests**
   - Test individual proximity sensors
   - Test IMU data streaming
   - Test individual finger motors

3. **Calibration**
   - Calibrate proximity sensors
   - Calibrate IMU orientation

4. **Integration Tests**
   - Monitor all sensors together
   - Record and analyze sensor data

## Individual Component Testing

### I2C Multiplexers

The I2C multiplexers (TCA9548A) are critical for connecting multiple VL6180X sensors. Test them first:

```bash
# Scan for multiplexers and connected devices
python -m hardware_tools.test_i2c_mux --scan

# Test a specific channel
python -m hardware_tools.test_i2c_mux --channel 0
```

Expected output:
```
Scanning I2C bus 1...
Found multiplexer at address 0x70
Scanning channels on multiplexer 0x70:
  Channel 0: Found devices at addresses [0x29, 0x30]
  Channel 1: Found devices at addresses [0x29]
  ...
```

Troubleshooting:
- If no multiplexer is found, check wiring and power supply
- If multiplexer is found but no devices on channels, check sensor connections

### VL6180X Proximity Sensors

Once multiplexers are confirmed working, test the VL6180X sensors:

```bash
# Test sensor on specific channel
python -m hardware_tools.test_vl6180x --mux-address 0x70 --channel 0

# Continuous reading mode
python -m hardware_tools.test_vl6180x --mux-address 0x70 --channel 0 --continuous

# Test all sensors
python -m hardware_tools.test_vl6180x --scan-all
```

Verify each sensor responds with reasonable distance measurements (0-255mm). The continuous mode is useful for verifying sensor behavior by moving objects in front of the sensor.

### IMU

Test the Microstrain IMU communication and data streaming:

```bash
# Display IMU information
python -m hardware_tools.test_imu --info

# Stream IMU data
python -m hardware_tools.test_imu --stream --duration 10

# Stream with plotting
python -m hardware_tools.test_imu --stream --plot
```

While streaming, move the IMU to verify orientation data changes appropriately. Check for consistent data rates and no interruptions in the data stream.

### Ability Hand Motors

Test the Ability Hand motor control and feedback:

```bash
# Show motor status
python -m hardware_tools.test_motor --status

# Test thumb position control
python -m hardware_tools.test_motor --finger 0 --position 45

# Test index finger current control
python -m hardware_tools.test_motor --finger 1 --current 0.3

# Run test sequence on all fingers
python -m hardware_tools.test_motor --sequence
```

Verify that:
- All motors respond to commands
- Position feedback matches commands
- Current control works properly
- No errors or faults are reported

## Calibration Procedures

### Proximity Sensor Calibration

Calibrate all proximity sensors to ensure accurate distance measurements:

```bash
# Automatic calibration sequence
python -m hardware_tools.calibrate_sensors --auto

# Manual calibration (interactive)
python -m hardware_tools.calibrate_sensors --manual

# Save calibration data
python -m hardware_tools.calibrate_sensors --auto --save
```

During calibration:
1. Position a flat object at specified distances (5mm, 10mm, 20mm, 30mm, 40mm, 60mm, 80mm)
2. Follow prompts to collect samples at each distance
3. Review calibration results
4. Save calibration data for use by the control system

### IMU Calibration

Calibrate the IMU to establish correct orientation reference:

```bash
# Automatic calibration sequence
python -m hardware_tools.calibrate_imu --auto

# Manual calibration (interactive)
python -m hardware_tools.calibrate_imu --manual

# Save calibration data
python -m hardware_tools.calibrate_imu --auto --save
```

During calibration:
1. Place IMU in specified orientations:
   - upright (palm facing forward)
   - inverted (upside down, palm facing backward)
   - palm_down (hand horizontal, palm facing down)
   - palm_up (hand horizontal, palm facing up)
2. Hold the IMU steady in each position
3. Review calibration results
4. Save calibration data for use by the control system

## Integration Testing

After testing individual components and performing calibrations, test all components together:

```bash
# Monitor all sensors
python -m hardware_tools.monitor_sensors

# Monitor specific sensors with plotting
python -m hardware_tools.monitor_sensors --sensors proximity,imu --plot

# Monitor with logging
python -m hardware_tools.monitor_sensors --log
```

During integration testing:
1. Move fingers and verify proximity sensor readings change
2. Change hand orientation and verify IMU data updates
3. Test basic grasping movements and observe all sensor responses

## Data Collection and Analysis

For detailed data collection and analysis:

```bash
# Log all sensor data for 60 seconds
python -m hardware_tools.logger_tool --duration 60

# Log specific sensors in CSV format
python -m hardware_tools.logger_tool --sensors proximity,imu --format csv --output proximity_imu_test.csv

# Log with faster sampling rate
python -m hardware_tools.logger_tool --interval 0.01 --duration 30
```

Use the collected data to:
- Analyze sensor noise and stability
- Verify timing and synchronization between sensors
- Identify potential issues before full system testing
- Tune control algorithms based on real sensor data

## Integration Steps

1. **Basic Hardware Setup**
   - Connect I2C multiplexers to the Raspberry Pi
   - Connect VL6180X sensors to multiplexer channels
   - Connect IMU via USB/UART
   - Connect Ability Hand via USB

2. **Individual Testing** (as outlined above)
   - Test each component separately
   - Document any issues or limitations

3. **Calibration** (as outlined above)
   - Perform all calibration procedures
   - Verify calibration results

4. **Low-Level Integration**
   - Run the monitor tool to view all sensors together
   - Verify sensors don't interfere with each other
   - Check for timing issues or data loss

5. **Control System Integration**
   - Update system configuration to use calibration data
   - Run main control system with hardware
   - Start with limited functionality (e.g., only monitoring)
   - Gradually enable actuator control

6. **Validation and Testing**
   - Perform systematic testing of grasp scenarios
   - Collect data logs for analysis
   - Fine-tune parameters as needed

## Troubleshooting

### I2C Issues

- **Problem**: Cannot detect multiplexers or sensors
  - Check I2C bus number (`i2cdetect -l`)
  - Verify wiring and connections
  - Check power supply to sensors
  - Try different I2C speed (`--bus-speed` option)

- **Problem**: Sensors detected but readings unstable
  - Check for proper grounding
  - Verify pull-up resistors are installed
  - Try shorter cable lengths
  - Check for EMI sources nearby

### IMU Issues

- **Problem**: Cannot connect to IMU
  - Check serial port permissions
  - Verify correct port is specified
  - Try different baud rates

- **Problem**: IMU data unstable
  - Ensure IMU is mounted securely
  - Check for magnetic interference
  - Recalibrate IMU

### Ability Hand Issues

- **Problem**: Cannot connect to Ability Hand
  - Verify hand is powered on
  - Check USB connection and permissions
  - Try different baud rate
  - Update firmware if available

- **Problem**: Motors unresponsive
  - Check error codes returned by hand
  - Verify current limits are set appropriately
  - Test each motor individually

### Integration Issues

- **Problem**: System works but operates slowly
  - Check CPU usage during operation
  - Monitor timing of sensor reads
  - Consider adjusting sampling rates
  - Optimize thread synchronization

- **Problem**: Data loss between components
  - Increase buffer sizes
  - Check thread priorities
  - Monitor system resources

## Next Steps

After hardware integration is complete and verified, proceed to:

1. **System Testing**
   - Run comprehensive system tests
   - Verify proper behavior in all grasp scenarios
   - Test error recovery

2. **Performance Optimization**
   - Fine-tune control parameters
   - Optimize sensor sampling rates
   - Improve response time

3. **User Testing**
   - Conduct user trials with the prosthetic
   - Gather feedback
   - Make adjustments based on user experience