import smbus

bus = smbus.SMBus(1)  # I2C bus 1
arduino_address = 0x08  # Arduino I2C address

def int16_to_bytes(val):
    val = int(val)
    if val < 0:
        val = (1 << 16) + val
    high = (val >> 8) & 0xFF
    low = val & 0xFF
    return [high, low]  




    data = int16_to_bytes(x1) + int16_to_bytes(y1) + int16_to_bytes(x2) + int16_to_bytes(y2)
    try:
        bus.write_i2c_block_data(arduino_address, 0x00, data)
        print(f"Sent: ({x1}, {y1}), ({x2}, {y2})")
    except Exception as e:
        print(f"I2C Send Error: {e}")

#send_two_points_16bit(0,0,0,0)
send_two_points_16bit(-123, 422, 345, -888)
#send_two_points_16bit(1,1,1,1)
#send_two_points_16bit(2, 9, 100, 3)

