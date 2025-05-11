# Hardware Testing Tools

This directory contains command-line tools for directly testing and calibrating hardware components of the prosthetic control system. These tools allow you to verify the functionality of individual components before integrating them into the full system.

## Ability Hand Integration

The system interfaces with the PSYONIC Ability Hand prosthetic through the `ability-hand-api` included in the root of the project. This API provides a Python wrapper for communicating with the hand.

### API Location and Structure

The Ability Hand API is included directly in the project at:
```
/ability-hand-api/
```

Key files and directories:
- `python/ah_wrapper/` - Core Python wrapper modules
- `python/ah_wrapper/ah_serial_client.py` - Main client for communication
- `python/docs/` - Documentation for the API
- `Documentation/ABILITY-HAND-ICD.pdf` - Interface Control Document

### Integration Method

The `AbilityHandInterface` class in `prosthetic_control_system/hand/ability_hand_interface.py` interfaces with the API by importing:

```python
from ah_wrapper.ah_serial_client import AHSerialClient
```

This import resolution works because we ensure the API path is in the Python path. The system is designed to gracefully fallback to a simulated mode if the API is not available.

### Enabling UART and Byte Stuffing

Before using the hardware tools with the Ability Hand, you must enable UART communication and byte stuffing using the PSYONIC App:

1. Connect to the hand in the PSYONIC App
2. Navigate to: Scan → SELECT HAND → Gear Icon ⚙️ → Troubleshoot → Developer Mode
3. Issue these commands individually:
   - We16
   - We46
   - We47

## Tools Overview

| Tool | Description | Hardware Required |
|------|-------------|-------------------|
| `test_i2c_mux.py` | Tests I2C multiplexers and scans for connected devices | I2C multiplexers (TCA9548A) |
| `test_vl6180x.py` | Tests VL6180X proximity sensors directly | VL6180X sensors |
| `test_imu.py` | Tests IMU connectivity and data streaming | Microstrain IMU |
| `test_motor.py` | Tests motor control and feedback | Ability Hand |
| `calibrate_sensors.py` | Calibrates proximity sensors | VL6180X sensors |
| `calibrate_imu.py` | Calibrates IMU orientation | Microstrain IMU |
| `monitor_sensors.py` | Real-time monitoring of all sensors | All sensors |
| `logger_tool.py` | Records sensor data to log files | Any sensors |

## Usage Instructions

### Prerequisites

1. Connect the hardware components to the appropriate ports:
   - I2C multiplexers: Connect to I2C bus
   - VL6180X sensors: Connect to multiplexers
   - IMU: Connect via USB/UART
   - Ability Hand: Connect via USB serial port

2. Install any required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Run the tools from the main directory:
   ```
   python -m hardware_tools.TOOL_NAME [options]
   ```

### Testing Workflow

For systematic hardware integration, follow this testing sequence:

1. **Hardware Connectivity Tests**
   ```
   python -m hardware_tools.test_i2c_mux --scan
   python -m hardware_tools.test_imu --info
   python -m hardware_tools.test_motor --status
   ```

2. **Component-Level Tests**
   ```
   python -m hardware_tools.test_vl6180x --channel 0 --continuous
   python -m hardware_tools.test_imu --stream
   python -m hardware_tools.test_motor --finger 0 --position 45
   ```

3. **Calibration**
   ```
   python -m hardware_tools.calibrate_sensors
   python -m hardware_tools.calibrate_imu
   ```

4. **Integrated Monitoring**
   ```
   python -m hardware_tools.monitor_sensors
   ```

## Detailed Tool Descriptions

### test_i2c_mux.py

Tests I2C multiplexers and scans for connected devices.

```
Usage: python -m hardware_tools.test_i2c_mux [options]

Options:
  --bus INT              I2C bus number (default: 1)
  --address HEX          I2C address of multiplexer (default: 0x73)
  --scan                 Scan all channels for devices
  --channel INT          Select specific channel (0-7)
  --check-both           Check both multiplexers on addresses 0x73 and 0x77
  --check-mux            Same as --check-both (legacy option)
  --verbose              Verbose output
```

Example:
```
# Scan all channels on default multiplexer (0x73)
python -m hardware_tools.test_i2c_mux --scan

# Check both multiplexers at 0x73 and 0x77
python -m hardware_tools.test_i2c_mux --check-both

# Test specific channel on a specific multiplexer
python -m hardware_tools.test_i2c_mux --address 0x77 --channel 0
```

### test_vl6180x.py

Tests VL6180X proximity sensors directly.

```
Usage: python -m hardware_tools.test_vl6180x [options]

Options:
  --bus INT              I2C bus number (default: 1)
  --address HEX          I2C address of sensor (default: 0x29)
  --mux-address HEX      I2C address of multiplexer (default: None)
  --channel INT          Multiplexer channel (0-7)
  --continuous           Continuous reading mode
  --samples INT          Number of samples to take (default: 10)
  --interval FLOAT       Sampling interval in seconds (default: 0.1)
  --scan-all             Scan both multiplexers (0x73 and 0x77) for all VL6180X sensors and read from them
  --plot                 Plot histogram of readings (requires matplotlib)
```

Example:
```
# Test sensor on channel 0 of multiplexer 0x73
python -m hardware_tools.test_vl6180x --mux-address 0x73 --channel 0

# Continuous reading mode
python -m hardware_tools.test_vl6180x --mux-address 0x73 --channel 0 --continuous

# Scan and monitor all VL6180X sensors on both multiplexers
python -m hardware_tools.test_vl6180x --scan-all
```

### test_imu.py

Tests IMU connectivity and data streaming.

```
Usage: python -m hardware_tools.test_imu [options]

Options:
  --port STRING          Serial port of IMU
  --baud INT             Baud rate (default: 115200)
  --info                 Display device information
  --stream               Stream IMU data
  --duration INT         Stream duration in seconds (default: 10)
  --raw                  Show raw data
```

Example:
```
# Get IMU information
python -m hardware_tools.test_imu --info

# Stream data for 10 seconds
python -m hardware_tools.test_imu --stream --duration 10
```

### test_motor.py

Tests motor control and feedback for the Ability Hand.

```
Usage: python -m hardware_tools.test_motor [options]

Options:
  --port STRING          Serial port of Ability Hand (auto-detected if not specified)
  --baud INT             Baud rate (default: 460800)
  --status               Show motor status
  --finger INT           Finger index (0: Thumb, 1: Index, 2: Middle, 3: Ring, 4: Pinky, 5: ThumbRotate)
  --position FLOAT       Set position (0-100 degrees)
  --current FLOAT        Set current (0-0.6 A)
  --sequence             Run test sequence
  --plot                 Show real-time plot of motor data
  --duration FLOAT       Duration in seconds (default: 5)
```

Example:
```
# Show motor status
python -m hardware_tools.test_motor --status

# Position control of thumb
python -m hardware_tools.test_motor --finger 0 --position 45

# Current control of index finger
python -m hardware_tools.test_motor --finger 1 --current 0.3

# Run test sequence with plotting
python -m hardware_tools.test_motor --sequence --plot
```

### calibrate_sensors.py

Calibrates proximity sensors.

```
Usage: python -m hardware_tools.calibrate_sensors [options]

Options:
  --auto                 Automatic calibration
  --manual               Manual calibration
  --save                 Save calibration data
```

Example:
```
# Automatic calibration
python -m hardware_tools.calibrate_sensors --auto

# Manual calibration with saving
python -m hardware_tools.calibrate_sensors --manual --save
```

### calibrate_imu.py

Calibrates IMU orientation.

```
Usage: python -m hardware_tools.calibrate_imu [options]

Options:
  --port STRING          Serial port of IMU
  --auto                 Automatic calibration
  --save                 Save calibration data
```

Example:
```
# Automatic calibration with saving
python -m hardware_tools.calibrate_imu --auto --save
```

### monitor_sensors.py

Real-time monitoring of all sensors.

```
Usage: python -m hardware_tools.monitor_sensors [options]

Options:
  --sensors STRING       Comma-separated list of sensors to monitor
  --interval FLOAT       Sampling interval in seconds (default: 0.05)
  --log                  Log data to file
  --plot                 Show real-time plot
```

Example:
```
# Monitor all sensors
python -m hardware_tools.monitor_sensors

# Monitor specific sensors with plotting
python -m hardware_tools.monitor_sensors --sensors proximity,imu --plot
```

### logger_tool.py

Records sensor data to log files.

```
Usage: python -m hardware_tools.logger_tool [options]

Options:
  --sensors STRING       Comma-separated list of sensors to log
  --duration INT         Recording duration in seconds
  --format STRING        Log format (json, csv) (default: json)
  --output STRING        Output file
```

Example:
```
# Log all sensor data for 60 seconds
python -m hardware_tools.logger_tool --duration 60

# Log specific sensors in CSV format
python -m hardware_tools.logger_tool --sensors proximity,imu --format csv
```

## Integration Testing

After verifying individual components, use the integrated monitoring tool to test all components together:

```
python -m hardware_tools.monitor_sensors --log
```

This will display real-time data from all sensors and log the data to a file for later analysis. Monitor the data to verify that all components are functioning correctly together before launching the full controller.

## Troubleshooting

### Ability Hand Connection Issues

1. Verify the hand is powered on
2. Check USB connection and permissions
3. Try different baud rates (default: 460800)
4. Ensure UART and byte stuffing are enabled via the app (We16, We46, We47 commands)
5. Check if the correct serial port is being used

### I2C Issues

1. Check I2C bus number (`i2cdetect -l`)
2. Verify wiring and connections
3. Check power supply to sensors
4. Try different I2C speeds with `--bus-speed` option

### Permission Issues on Linux

If you have permission issues accessing hardware on Linux, add your user to the appropriate groups:

```bash
sudo usermod -a -G dialout,i2c $USER
# Log out and log back in for changes to take effect
```