#!/usr/bin/env python3
# Simple movement test for Ability Hand
# Based directly on the approach from the ability-hand-api repo

import sys
import time
import os

def main():
    print("==== Ability Hand Simple Movement Test ====")
    print("Attempting to import AHSerialClient...")
    
    try:
        # Try to import from the ability-hand-api
        from ah_wrapper.ah_serial_client import AHSerialClient
        
        # Create client at higher baud rate for more responsive control
        print("Connecting to Ability Hand...")
        client = AHSerialClient(baud_rate=460800)
        print("Connected!")
        
        # Wave pattern
        print("\nStarting wave pattern - press Ctrl+C to stop\n")
        
        try:
            # The wave motion
            while True:
                print("Opening hand...")
                # Set all fingers open
                client.set_position([0, 0, 0, 0, 0, 0])
                time.sleep(1)
                
                print("Closing index finger...")
                # Close just index finger
                client.set_position([80, 0, 0, 0, 0, 0])
                time.sleep(0.5)
                
                print("Closing middle finger...")
                # Close index and middle
                client.set_position([80, 80, 0, 0, 0, 0])
                time.sleep(0.5)
                
                print("Closing ring finger...")
                # Close index, middle, and ring
                client.set_position([80, 80, 80, 0, 0, 0])
                time.sleep(0.5)
                
                print("Closing pinky finger...")
                # Close index, middle, ring, and pinky
                client.set_position([80, 80, 80, 80, 0, 0])
                time.sleep(0.5)
                
                print("Closing thumb...")
                # Close all fingers
                client.set_position([80, 80, 80, 80, 80, 0])
                time.sleep(1)
                
                print("Rotating thumb...")
                # Close all fingers and rotate thumb
                client.set_position([80, 80, 80, 80, 80, -80])
                time.sleep(1)
                
                print("Making a fist...")
                # Full closed position
                client.set_position([100, 100, 100, 100, 100, -100])
                time.sleep(1.5)
                
                # Test torque control briefly
                print("Testing torque control...")
                client.set_torque([0.3, 0.3, 0.3, 0.3, 0.3, -0.3])
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        finally:
            # Reset to open position before closing
            print("Resetting to open position...")
            client.set_position([0, 0, 0, 0, 0, 0])
            time.sleep(0.5)
            
            # Close client connection
            client.close()
            print("Connection closed")
            
    except ImportError:
        print("\nERROR: Could not import AHSerialClient.")
        print("Please make sure ability-hand-api is installed and in your Python path.")
        print("\nInstallation instructions:")
        print("  1. git clone https://github.com/psyonicinc/ability-hand-api.git")
        print("  2. cd ability-hand-api/python")
        print("  3. pip3 install -r requirements.txt")
        print("  4. Execute We16, We46, We47 commands in PSYONIC app developer mode")
        print("\nTrying fallback to direct serial control...")
        
        try_fallback_serial()
        
    except Exception as e:
        print(f"\nERROR: {e}")
        print("Trying fallback to direct serial control...")
        
        try_fallback_serial()

def try_fallback_serial():
    """Try to control the hand using direct serial communication"""
    try:
        import serial
        import struct
        import time
        
        print("\nAttempting direct serial control...")
        
        # Try to find a USB serial device
        import glob
        serial_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        
        if not serial_ports:
            print("No serial ports found. Please connect the Ability Hand.")
            return
            
        port = serial_ports[0]
        print(f"Using serial port: {port}")
        
        # Open serial connection
        ser = serial.Serial(
            port=port,
            baudrate=460800,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        
        # Function to create position command directly
        def create_position_command(positions, reply_mode=0, addr=0x50):
            # Scale positions to 16-bit values
            scaled_positions = [int((p + 100) * 32767 / 200) for p in positions]
            
            # Create command bytes
            cmd = bytearray([addr, 0x00, reply_mode])
            
            # Add position data
            for pos in scaled_positions:
                cmd.extend(struct.pack('<H', pos))
                
            # Add checksum
            checksum = 0
            for b in cmd:
                checksum ^= b
            cmd.append(checksum)
            
            return cmd
            
        # Function to write command with byte stuffing
        def write_command(cmd):
            output = bytearray([0x7E])  # Start flag
            
            for b in cmd:
                if b in [0x7E, 0x7D]:
                    output.append(0x7D)  # Escape byte
                    output.append(b ^ 0x20)  # XOR with 0x20
                else:
                    output.append(b)
                    
            output.append(0x7E)  # End flag
            
            ser.write(output)
            
        try:
            print("\nStarting simplified wave pattern - press Ctrl+C to stop\n")
            
            # Simple test sequence
            positions = [
                [0, 0, 0, 0, 0, 0],     # All open
                [80, 0, 0, 0, 0, 0],    # Index closed
                [80, 80, 0, 0, 0, 0],   # Index, middle closed
                [80, 80, 80, 0, 0, 0],  # Index, middle, ring closed
                [80, 80, 80, 80, 0, 0], # Index, middle, ring, pinky closed
                [80, 80, 80, 80, 80, 0], # All fingers closed
                [100, 100, 100, 100, 100, -100], # Full fist
                [0, 0, 0, 0, 0, 0]      # All open again
            ]
            
            # Send each position
            for i, pos in enumerate(positions):
                print(f"Position {i+1}: {pos}")
                cmd = create_position_command(pos)
                write_command(cmd)
                time.sleep(1.5)
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
            
        finally:
            # Reset to open position
            cmd = create_position_command([0, 0, 0, 0, 0, 0])
            write_command(cmd)
            
            # Close serial connection
            ser.close()
            print("Serial connection closed")
            
    except ImportError:
        print("\nERROR: Could not import serial module.")
        print("Please install pyserial: pip3 install pyserial")
        
    except Exception as e:
        print(f"\nERROR during fallback serial control: {e}")
        print("Unable to control the hand.")

if __name__ == "__main__":
    main()