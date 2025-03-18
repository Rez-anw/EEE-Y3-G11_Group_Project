# Raspberry Pi Code (I2C Master + Continuous Control) 
# This script: Sends real-time movement commands to Arduino. Requests data from Arduino (servo angles).

import smbus
import time

# Initialize I2C bus
bus = smbus.SMBus(1)  # 1 for Raspberry Pi I2C bus
arduino_address = 0x08  # I2C address of Arduino

# Function to send movement command to Arduino
def send_command(command):
    try:
        bus.write_byte(arduino_address, ord(command))
        print(f"Sent command: {command}")
        time.sleep(0.1)  # Short delay
    except Exception as e:
        print(f"Error sending command: {e}")

# Function to read servo angles from Arduino
def read_servo_positions():
    try:
        data = bus.read_i2c_block_data(arduino_address, 0, 2)  # Read two bytes (X and Y)
        angleX, angleY = data[0], data[1]
        print(f"Servo Positions - X: {angleX}, Y: {angleY}")
    except Exception as e:
        print(f"Error reading servo positions: {e}")

# Continuous control loop
while True:
    command = input("Enter command (L, R, U, D, Q to quit): ").strip().upper()
    
    if command == 'Q':
        print("Exiting...")
        break
    
    if command in ['L', 'R', 'U', 'D']:
        send_command(command)
        time.sleep(0.2)
        read_servo_positions()
    else:
        print("Invalid command! Use L, R, U, D, or Q.")
