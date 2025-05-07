
import numpy as np
import wave
import serial
import serial.tools.list_ports

BAUD_RATE = 115200
SAMPLE_RATE = 10000

MANUAL_MODE = bytes(chr(0x8F), 'utf-8')
DISTANCE_MODE = bytes(chr(0x90), 'utf-8')

# Select the first com port
ports = serial.tools.list_ports.comports()
device = None
for i in ports:
    if i.description[0:3] == "STM":
        device = i

print(f"STM Serial Port = {device.device}")

ser = serial.Serial(device.device, BAUD_RATE)


data = []

for i in range(SAMPLE_RATE*5):
    ch = ser.read(1)
    print(ch)
    data.append(int.from_bytes(ch,byteorder="big"))


data = np.array(data)

data = (data - data.min()) / data.max()
data = data * 255

data = data.astype(np.uint8)

with wave.open("output.wav", 'wb') as wave_file:    
    wave_file.setnchannels(1)
    wave_file.setsampwidth(1)
    wave_file.setframerate(SAMPLE_RATE)
    wave_file.writeframes(data.tobytes())
