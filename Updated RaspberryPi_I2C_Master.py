# Raspberry Pi Code (I2C Master + Float Control)
# Sends 5 floats to Arduino and reads back 2-byte servo positions

import smbus
import struct
import time

# I2C setup
ARDUINO_ADDRESS = 0x08
bus = smbus.SMBus(1)

def send_floats(float_array):
    """Send an array of 5 floats to Arduino as bytes."""
    byte_data = bytearray()
    for value in float_array:
        byte_data.extend(struct.pack('f', value))  # 4 bytes per float

    try:
        bus.write_i2c_block_data(ARDUINO_ADDRESS, 0, list(byte_data))
        print(f"Sent: {float_array}")
    except Exception as e:
        print(f"Error sending floats: {e}")

def read_servo_positions():
    """Read 2 bytes from Arduino representing current servo angles."""
    try:
        data = bus.read_i2c_block_data(ARDUINO_ADDRESS, 0, 2)
        angleX, angleY = data[0], data[1]
        print(f"Servo Positions - X: {angleX}, Y: {angleY}")
    except Exception as e:
        print(f"Error reading servo positions: {e}")

# Main loop
while True:
    try:
        # Get user input for testing
        x_input = float(input("Enter X value (-1.0 to 1.0): "))
        y_input = float(input("Enter Y value (-1.0 to 1.0): "))

        # Optional: Clamp to expected range
        x_input = max(min(x_input, 1.0), -1.0)
        y_input = max(min(y_input, 1.0), -1.0)

        # Create float array to send: [x1, y1, x2, y2, q]
        data_to_send = [x_input, y_input, 0.0, 0.0, 0.0]

        send_floats(data_to_send)
        time.sleep(0.1)
        read_servo_positions()
        time.sleep(0.2)

    except KeyboardInterrupt:
        print("Exiting...")
        break
    except ValueError:
        print("Invalid input. Please enter numeric values.")
