import serial.tools.list_ports

import serial
import time

ports = serial.tools.list_ports.comports()
for port in ports:
    print(port.device)

ser = serial.Serial("COM6", 115200, timeout=2)  # Update COM port as needed
time.sleep(2)  # Let the board boot

for x in range(255):
    ser.write(b'E15')  # Send test message (if STM is receiving)
    data = ser.readline()
    print(f"Received: {data}")

ser.close()