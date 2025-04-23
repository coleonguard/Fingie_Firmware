# Prosthetic Hand Controller

This module implements a comprehensive system for controlling a prosthetic hand based on proximity sensor readings, with advanced control algorithms and data logging/replay capabilities.

## Theory of Operation

The controller implements a three-phase control system:

1. **Approach Phase** (distance > threshold):
   - No movement occurs when the hand is far from objects
   - Sensors are continuously monitored

2. **Proportional Control Phase** (threshold > distance > contact):
   - As the hand approaches an object, fingers begin to close
   - Closing force is proportional to proximity
   - MCP (metacarpophalangeal) joints are prioritized based on Index1 sensors

3. **Contact Phase** (distance < contact threshold):
   - Upon contact detection, the system switches to torque control
   - The thumb opposes the other fingers with balanced torque

The system uses Kalman filtering to smooth sensor readings and implements current-based motor control to approximate torque.

## System Architecture

The controller is organized into modular components:

- **sensors.py**: Manages proximity sensor readings and filtering
- **actuators.py**: Controls motor currents for finger movement
- **hand_controller.py**: Implements the main control algorithm
- **data_logger.py**: Records all sensor and control data
- **data_replay.py**: Replays and analyzes recorded sessions
- **main.py**: Command-line interface and application entry point

## Usage

### Running the Controller

```bash
./run_controller.py run [options]

Options:
  --rate RATE             Control rate in Hz (default: 20)
  --interval INTERVAL     Status display interval in seconds (default: 1.0)
  --no-logging            Disable data logging
  --no-imu                Disable IMU tracking
  --log-path LOG_PATH     Path to store log data (default: ~/hand_data)
  --snapshot-interval SEC Auto-snapshot interval in seconds (0 to disable)
```

### Data Logging

The system automatically logs all sensor readings, motor currents, and control modes during operation. Data is stored in both CSV and JSON formats for easy analysis. IMU orientation data is also captured if IMU sensors are available.

### Replaying Sessions

```bash
./run_controller.py replay SESSION [options]

Arguments:
  SESSION                 Session directory name or path

Options:
  --speed SPEED           Replay speed factor (default: 1.0)
  --no-replay             Skip actual replay (useful with --plot/--analyze)
  --plot                  Generate plots of the session
  --analyze               Analyze phase transitions
  --log-path LOG_PATH     Path to sessions directory (default: ~/hand_data)
```

### Listing Available Sessions

```bash
./run_controller.py list [--log-path LOG_PATH]
```

### Visualization Options

#### 2D Hand Visualization

The system provides detailed 2D visualizations of the hand model that can be generated as static images or animations:

```bash
# Generate static hand visualization
./run_controller.py visualize static [options]

Options:
  --params PARAMS_FILE     Path to custom hand parameters file
  --output OUTPUT_FILE     Output image file (default: hand_model_detailed.png)
```

```bash
# Generate hand animation from recorded data
./run_controller.py visualize animate DATA_FILE [options]

Arguments:
  DATA_FILE               Path to recorded data file (JSON or CSV)

Options:
  --output OUTPUT_FILE    Output video file (default: hand_animation.mp4)
  --fps FPS               Frames per second for video (default: 30)
  --duration DURATION     Optional duration limit in seconds
```

The visualization system features:
- Anatomically accurate hand proportions
- Realistic finger rendering with joints and segments
- Proximity sensor placement on finger pulp
- IMU sensor visualization
- Automatic finger movement based on sensor data
- Customizable hand parameters via JSON file

#### MuJoCo 3D Visualization

The system can also generate MuJoCo model files for 3D visualization and replay:

```bash
./run_controller.py mujoco [options]

Options:
  --type {default,relaxed,replay}  Type of model to generate (default: relaxed)
  --data-file DATA_FILE           Data file to use for replay model (required for replay type)
  --output-dir OUTPUT_DIR         Directory to save models (default: ~/mujoco_models)
  --output-name OUTPUT_NAME       Name for output model file (default: replay_model)
  --visualize                      Generate hand dimension visualization
```

To visualize the models with MuJoCo:

1. Install MuJoCo:
   ```bash
   pip install mujoco-py
   ```

2. View a static model (relaxed pose):
   ```bash
   python -c "import mujoco_py; sim = mujoco_py.MjSim(mujoco_py.load_model_from_path('~/mujoco_models/hand_model_relaxed.xml')); viewer = mujoco_py.MjViewer(sim); viewer.render()"
   ```

3. Replay recorded data:
   ```bash
   python -c "import mujoco_py; sim = mujoco_py.MjSim(mujoco_py.load_model_from_path('~/mujoco_models/replay_model.xml')); viewer = mujoco_py.MjViewer(sim); viewer.render()"
   ```

## Hardware Interface

The system is designed to work with:

- VL6180X proximity sensors for finger position sensing
- TCA9548A I²C multiplexers for managing multiple sensors
- Current-controlled motor drivers (implementation-specific)

## Customization

The system can be customized by modifying:

- Control thresholds in `hand_controller.py`
- Kalman filter parameters in `sensors.py`
- Motor control characteristics in `actuators.py`
- Adding additional sensors or motor channels

## Data Analysis

Recorded sessions can be analyzed using the built-in tools:

- Visualize sensor readings, motor currents, and control modes
- Analyze phase transitions and control performance
- Validate control algorithms through replay