import serial
import serial.tools.list_ports
import time

BAUD_RATE = 115200
STMS = 4

# Select the first com port
ports = serial.tools.list_ports.comports()
device = None
for i in ports:
    if i.description[0:3] == "STM":
        device = i

print(f"STM Serial Port = {device.device}")

ser = serial.Serial(device.device, BAUD_RATE)

def serial_write(data):
    global ser

    ser.write(data)

def serial_read_string(num):
    global ser

    return ser.read(num).decode()


def read_msg():

    global ser
    ch = 'a'
    data = []

    while ch != '\0':
        ch = ser.read(1).decode()
        data.append(ch)

    return ''.join(data)
# Transmit 'request signal'


while True:
    str = input("Enter the message to send: ")
    #serial_write(bytes(f'0E15{str}'+((STMS+3)*' ')+'\r\n\0', 'utf-8'))
    serial_write(bytes(f'0E15{str}\r\n\0', 'utf-8'))

    #print(serial_read_string(7+5+len(str)))
    print(read_msg())

