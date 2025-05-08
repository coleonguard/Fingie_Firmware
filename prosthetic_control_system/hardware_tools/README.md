# Hardware Testing Tools

This directory contains command-line tools for directly testing and calibrating hardware components of the prosthetic control system. These tools allow you to verify the functionality of individual components before integrating them into the full system.

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
  --address HEX          I2C address of multiplexer (default: 0x70)
  --scan                 Scan all channels for devices
  --channel INT          Select specific channel (0-7)
  --verbose              Verbose output
```

Example:
```
# Scan all channels for connected devices
python -m hardware_tools.test_i2c_mux --scan

# Test specific channel
python -m hardware_tools.test_i2c_mux --channel 0
```

### test_vl6180x.py

Tests VL6180X proximity sensors directly.

```
Usage: python -m hardware_tools.test_vl6180x [options]

Options:
  --bus INT              I2C bus number (default: 1)
  --address HEX          I2C address of sensor (default: 0x29)
  --mux-address HEX      I2C address of multiplexer (default: 0x70)
  --channel INT          Multiplexer channel (0-7)
  --continuous           Continuous reading mode
  --samples INT          Number of samples to take (default: 10)
  --interval FLOAT       Sampling interval in seconds (default: 0.1)
```

Example:
```
# Test sensor on channel 0 of multiplexer
python -m hardware_tools.test_vl6180x --channel 0

# Continuous reading mode
python -m hardware_tools.test_vl6180x --channel 0 --continuous
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

Tests motor control and feedback.

```
Usage: python -m hardware_tools.test_motor [options]

Options:
  --port STRING          Serial port of Ability Hand
  --baud INT             Baud rate (default: 115200)
  --status               Show motor status
  --finger INT           Finger index (0: Thumb, 1: Index, 2: Middle, 3: Ring, 4: Pinky)
  --position FLOAT       Set position (0-100 degrees)
  --current FLOAT        Set current (0-0.6 A)
  --sequence             Run test sequence
```

Example:
```
# Show motor status
python -m hardware_tools.test_motor --status

# Position control of thumb
python -m hardware_tools.test_motor --finger 0 --position 45

# Current control of index finger
python -m hardware_tools.test_motor --finger 1 --current 0.3
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