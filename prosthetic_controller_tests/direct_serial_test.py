#!/usr/bin/env python3
# Direct serial communication with Ability Hand
# Minimal implementation with no dependencies other than pyserial

import serial
import struct
import time
import glob
import sys

# Control parameters
HAND_ADDRESS = 0x50
BAUD_RATE = 460800
REPLY_MODE = 0  # 0: Pos,Cur,Touch, 1: Pos,Vel,Touch, 2: Pos,Cur,Vel

def find_serial_port():
    """Find available serial ports for Ability Hand"""
    # Try common USB serial device patterns
    patterns = ['/dev/ttyUSB*', '/dev/ttyACM*']
    ports = []
    
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    
    if ports:
        print(f"Found serial ports: {', '.join(ports)}")
        return ports[0]  # Return the first one
    else:
        print("No serial ports found.")
        return None

def create_position_command(positions, reply_mode=REPLY_MODE, addr=HAND_ADDRESS):
    """Create a position command for the Ability Hand
    
    Args:
        positions: List of 6 position values [-100 to 100]
        reply_mode: Feedback reply mode
        addr: Hand address
        
    Returns:
        Byte array with command (before byte stuffing)
    """
    # Ensure we have 6 positions
    if len(positions) != 6:
        raise ValueError("Must provide exactly 6 position values")
        
    # Scale positions to 16-bit values
    scaled_positions = []
    for p in positions:
        # Clamp to valid range
        p = max(-100, min(100, p))
        # Scale to 0-65535 range, centered at 32767
        scaled = int((p + 100) * 32767 / 200)
        scaled_positions.append(scaled)
    
    # Create command bytes [address, command, reply_mode, data...]
    cmd = bytearray([addr, 0x00, reply_mode])  # 0x00 = position command
    
    # Add position data (little-endian 16-bit values)
    for pos in scaled_positions:
        cmd.extend(struct.pack('<H', pos))
        
    # Add XOR checksum
    checksum = 0
    for b in cmd:
        checksum ^= b
    cmd.append(checksum)
    
    return cmd

def create_torque_command(currents, reply_mode=REPLY_MODE, addr=HAND_ADDRESS):
    """Create a torque command for the Ability Hand
    
    Args:
        currents: List of 6 current values [-1.0 to 1.0]
        reply_mode: Feedback reply mode
        addr: Hand address
        
    Returns:
        Byte array with command (before byte stuffing)
    """
    # Ensure we have 6 current values
    if len(currents) != 6:
        raise ValueError("Must provide exactly 6 current values")
        
    # Scale currents to 16-bit values
    scaled_currents = []
    for c in currents:
        # Clamp to valid range
        c = max(-1.0, min(1.0, c))
        # Scale to 0-65535 range, centered at 32767
        scaled = int((c + 1.0) * 32767 / 2.0)
        scaled_currents.append(scaled)
    
    # Create command bytes [address, command, reply_mode, data...]
    cmd = bytearray([addr, 0x02, reply_mode])  # 0x02 = current/torque command
    
    # Add current data (little-endian 16-bit values)
    for curr in scaled_currents:
        cmd.extend(struct.pack('<H', curr))
        
    # Add XOR checksum
    checksum = 0
    for b in cmd:
        checksum ^= b
    cmd.append(checksum)
    
    return cmd

def apply_byte_stuffing(cmd):
    """Apply PPP-like byte stuffing to command
    
    Args:
        cmd: Raw command byte array
        
    Returns:
        Byte-stuffed command with start/end flags
    """
    output = bytearray([0x7E])  # Start flag
    
    for b in cmd:
        if b in [0x7E, 0x7D]:
            output.append(0x7D)  # Escape byte
            output.append(b ^ 0x20)  # XOR with 0x20
        else:
            output.append(b)
            
    output.append(0x7E)  # End flag
    
    return output

def send_command(ser, cmd):
    """Send a command to the Ability Hand
    
    Args:
        ser: Serial connection
        cmd: Raw command (will be byte-stuffed)
    """
    # Apply byte stuffing
    stuffed_cmd = apply_byte_stuffing(cmd)
    
    # Send to hand
    ser.write(stuffed_cmd)
    
    # Small delay to ensure the command is processed
    time.sleep(0.01)

def test_sequence(ser):
    """Run a test sequence of movements
    
    Args:
        ser: Serial connection
    """
    try:
        print("\n=== Running test sequence ===")
        print("Press Ctrl+C at any time to stop\n")
        
        # Zero all fingers
        print("1. Opening hand...")
        cmd = create_position_command([0, 0, 0, 0, 0, 0])
        send_command(ser, cmd)
        time.sleep(1.5)
        
        # Close index finger
        print("2. Closing index finger...")
        cmd = create_position_command([80, 0, 0, 0, 0, 0])
        send_command(ser, cmd)
        time.sleep(1)
        
        # Close middle finger too
        print("3. Closing middle finger...")
        cmd = create_position_command([80, 80, 0, 0, 0, 0])
        send_command(ser, cmd)
        time.sleep(1)
        
        # Close ring finger too
        print("4. Closing ring finger...")
        cmd = create_position_command([80, 80, 80, 0, 0, 0])
        send_command(ser, cmd)
        time.sleep(1)
        
        # Close pinky finger too
        print("5. Closing pinky finger...")
        cmd = create_position_command([80, 80, 80, 80, 0, 0])
        send_command(ser, cmd)
        time.sleep(1)
        
        # Close thumb too
        print("6. Closing thumb...")
        cmd = create_position_command([80, 80, 80, 80, 80, 0])
        send_command(ser, cmd)
        time.sleep(1)
        
        # Rotate thumb
        print("7. Rotating thumb...")
        cmd = create_position_command([80, 80, 80, 80, 80, -80])
        send_command(ser, cmd)
        time.sleep(1.5)
        
        # Make a fist
        print("8. Making a fist...")
        cmd = create_position_command([100, 100, 100, 100, 100, -100])
        send_command(ser, cmd)
        time.sleep(2)
        
        # Try torque control
        print("9. Testing torque control...")
        cmd = create_torque_command([0.3, 0.3, 0.3, 0.3, 0.3, -0.3])
        send_command(ser, cmd)
        time.sleep(2)
        
        # Back to position control, open hand
        print("10. Opening hand...")
        cmd = create_position_command([0, 0, 0, 0, 0, 0])
        send_command(ser, cmd)
        time.sleep(1.5)
        
        print("\nTest sequence complete.")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        
        # Ensure hand is opened before exiting
        print("Opening hand...")
        cmd = create_position_command([0, 0, 0, 0, 0, 0])
        send_command(ser, cmd)

def main():
    """Main function to run the test"""
    print("==== Ability Hand Direct Serial Test ====")
    
    # First, make sure we have pyserial
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed.")
        print("Please install with: pip install pyserial")
        return
    
    # Find a serial port
    port = find_serial_port()
    if not port:
        print("No serial port found. Please connect the Ability Hand.")
        return
        
    try:
        # Open the serial connection
        print(f"Opening serial port {port} at {BAUD_RATE} baud...")
        ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        
        print("Serial port opened successfully.")
        
        # Run the test sequence
        test_sequence(ser)
        
    except serial.SerialException as e:
        print(f"ERROR opening serial port: {e}")
        print("Please check connections and permissions.")
        print("You may need to run: sudo usermod -a -G dialout $USER")
        
    except Exception as e:
        print(f"ERROR: {e}")
        
    finally:
        # Close the serial port if it was opened
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()