import serial
import serial.tools.list_ports

BAUD_RATE = 115200

# Select the first com port
device = serial.tools.list_ports.comports()[0]

print(f"STM Serial Port = {device.device}")

ser = serial.Serial(device.device, 115200)

def serial_write(data):
    global ser

    ser.write(data)

def serial_read_string(num):
    global ser

    return ser.read(num).decode()

# Transmit 'request signal'
serial_write(b'E15')

while True:

    print(serial_read_string(16))
