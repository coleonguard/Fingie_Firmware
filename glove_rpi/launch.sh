#!/bin/bash

# Function to start a sensor script if specified
start_sensor() {
    sensor=$1
    script_name="${sensor}_sensor.py"
    if [ -f "$script_name" ]; then
        echo "Starting $sensor Sensor script..."
        python3 "$script_name" &
    else
        echo "Script for $sensor not found: $script_name"
    fi
}

# Flags to track which sensors to start
start_proximity=false
start_pressure=false
start_emg=false
start_imu=false

# Check if any arguments are passed
if [ "$#" -eq 0 ]; then
    # No arguments, start all sensors
    echo "No arguments provided. Starting all sensors..."
    start_proximity=true
    start_pressure=true
    start_emg=true
    start_imu=true
else
    # Process arguments to determine which sensors to start
    for arg in "$@"; do
        case $arg in
            proximity)
                start_proximity=true
                ;;
            pressure)
                start_pressure=true
                ;;
            emg)
                start_emg=true
                ;;
            imu)
                start_imu=true
                ;;
            *)
                echo "Unknown sensor: $arg. Valid options are: proximity, pressure, emg, imu."
                ;;
        esac
    done
fi

# Start each sensor based on the flags
if [ "$start_proximity" = true ]; then
    start_sensor "proximity"
fi
if [ "$start_pressure" = true ]; then
    start_sensor "pressure"
fi
if [ "$start_emg" = true ]; then
    start_sensor "emg"
fi
if [ "$start_imu" = true ]; then
    start_sensor "imu"
fi

# Wait to ensure selected sensors are running
sleep 2

# Prepare list of sensors to transmit as an array
sensors_to_transmit=()
[ "$start_proximity" = true ] && sensors_to_transmit+=("proximity")
[ "$start_pressure" = true ] && sensors_to_transmit+=("pressure")
[ "$start_emg" = true ] && sensors_to_transmit+=("emg")
[ "$start_imu" = true ] && sensors_to_transmit+=("imu")

# Debug: Print sensors to transmit
echo "Sensors to transmit: ${sensors_to_transmit[@]}"

# Launch the transmission script with sensor arguments
echo "Starting Transmission script..."
python3 transmission_script.py "${sensors_to_transmit[@]}"

