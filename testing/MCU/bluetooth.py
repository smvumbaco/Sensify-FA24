import serial
import time

port = 'COM3'
baudrate = 115200

def decode_packet(packet):
    marker = packet[0]
    address = packet[1]
    length = packet[2]
    data = packet[3:]

    print(f"Marker: {hex(marker)}")
    print(f"Sensor Address: {address}")
    print(f"Data Length: {length}")
    print(f"Sensor Data: {list(data)}")

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {port} at {baudrate} bps")

    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            if data[0] == 0xFF:
                decode_packet(data)
                time.sleep(0.1)

        ser.write(b'1')

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
