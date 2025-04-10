import serial
import serial.tools.list_ports 

word = b"word\n"
DELAY_BETWEEN_REQUESTS = 0.1
TIMEOUT = 1

devices = serial.tools.list_ports.comports()

for device in devices:
    print(device.device)

ser = serial.Serial(devices[0].device,115200,timeout=None) #depends on the port that is connected
# send string to stm
ser.write(word)

#wait for response and print to console
response = ser.readline()
if response:
        decoded = response.decode().strip() #.strip gets rid of white spaces in the string, eg. \n
        print(decoded)

ser.close()

