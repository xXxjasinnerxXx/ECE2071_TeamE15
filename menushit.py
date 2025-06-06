
import numpy as np
import wave
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import os
import csv
import time

BAUD_RATE = 921600
SAMPLE_RATE = 44100

MANUAL_MODE = bytes(chr(0x8F), 'utf-8')
DISTANCE_MODE = bytes(chr(0x90), 'utf-8')

# Select the first com port
ports = serial.tools.list_ports.comports()
device = None
for i in ports:
    if i.description[0:3] == "STM":
        device = i

print(f"STM Serial Port = {device.device}\n")

ser = serial.Serial(device.device, BAUD_RATE)


data = []
def manual_trigger(data = None):
    
    global ser

    print("You are in Manual trigger mode: Press ctrl + C for menu\n")
    
    try:
        ser.write(MANUAL_MODE)
        ser.write(bytes(chr(0), 'utf-8'))

        data = []
        while True:
            try:
                snippetLength = int(input("Enter length of snippet in seconds: "))
                break
            except ValueError:
                print("Invalid value, try again")
                continue
                
        # for i in range(SAMPLE_RATE*snippetLength):
        #     ch = ser.read(1)
        #     data.append(int.from_bytes(ch,byteorder="big",signed=False))

        ser.flush()
        ser.reset_input_buffer()

        for _ in range(SAMPLE_RATE * snippetLength):
            raw = ser.read(2)

            data.append(raw)    

        for i in range(len(data)):
            data[i] = int.from_bytes(data[i], byteorder="little", signed=False)  # STM32 uses little-endian by default
            data[i] &= 0x0FFF        

        data = np.array(data, dtype=np.uint16)
        
        #data = np.array(data)
        #data = (data - data.min()) / data.max()
        #data = data * 255 

        #data = data.astype(np.uint8)
        return data
        
    except KeyboardInterrupt:
        return None
    
def distance_trigger():

    global ser

    print("You are in Distance Trigger Mode: Press ctrl + C for menu\n")

    while True:
        try:
            distance = int(input("Enter the target distance (between 5 and 30): "))
            if (5 <= distance <= 30):
                break
            else:
                print("Distance must be between 5 and 30")

        except ValueError:
            print("Enter a valid distance")

    ser.write(DISTANCE_MODE)
    ser.write(bytes(chr(distance), 'utf-8'))


    data = []
    zeroCount = 0

    startTime = time.time()

    print("Started Recording...")
    try:
        while True:
            raw = ser.read(2)
            
            if raw == b'\x00\x00':
                zeroCount += 1
            
            else:
                data.append(raw)

            if zeroCount > 5000:
                zeroCount = 0
                print("Stopped recording...")
                startTime = time.time()
                while ser.read(2) == b'\x00\x00':
                    if time.time() - startTime >= 10: # 10 seconds
                        print("Stopped recording due to 10 second timeout")
                        raise TimeoutError

                print("Started Recording...")
                startTime = time.time()

    except TimeoutError:
        pass

    except KeyboardInterrupt:
        print("Stopped recording due to keyboard interrupt")

    for i in range(len(data)):
        data[i] = int.from_bytes(data[i], byteorder="little", signed=False)  # STM32 uses little-endian by default
        data[i] &= 0x0FFF        

    data = np.array(data, dtype=np.uint16)

    return data
    
def menu():
    """
    Function to run basic menu functionality
    """
    data = None
    try:    
        while True:

            print("""

███████╗████████╗███╗   ███╗    ███████╗ ██████╗ ██╗   ██╗███╗   ██╗██████╗ ███████╗
██╔════╝╚══██╔══╝████╗ ████║    ██╔════╝██╔═══██╗██║   ██║████╗  ██║██╔══██╗██╔════╝
███████╗   ██║   ██╔████╔██║    ███████╗██║   ██║██║   ██║██╔██╗ ██║██║  ██║███████╗
╚════██║   ██║   ██║╚██╔╝██║    ╚════██║██║   ██║██║   ██║██║╚██╗██║██║  ██║╚════██║
███████║   ██║   ██║ ╚═╝ ██║    ███████║╚██████╔╝╚██████╔╝██║ ╚████║██████╔╝███████║
╚══════╝   ╚═╝   ╚═╝     ╚═╝    ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═══╝╚═════╝ ╚══════╝
                                                                                    
1 > Manual Triggering Mode
2 > Distance Triggering Mode 
3 > Data Analysis Mode
99 > Exit Program (Ctrl + C)              
""")

            decision = input("Enter a mode: ")
            if decision == "1":
                data = manual_trigger()
            elif decision == "2":
                data = distance_trigger()
            elif decision == "3":
                output_type(data)
            elif decision == "99":
                print("Thank you for using STM SOUNDS")
                exit()
            else:
                print("invalid input, try again\n")
                continue
    except KeyboardInterrupt:
        print("\nExiting program")
        exit()

def output_type(data):
    if len(data) == 0:
        print("no data")
        return None
    data = np.array(data)
    while True: 
            outputChoice = input("Enter output type:\nwav, png or csv: ")
            if outputChoice == "wav":

                data = data.astype(np.int16)
                data = ((data - 2048) / 2047.0) * 32767  # center and scale
                data = np.clip(data, -32768, 32767).astype(np.int16)

                with wave.open("output.wav", 'wb') as wave_file:    
                    wave_file.setnchannels(1)
                    wave_file.setsampwidth(2)
                    wave_file.setframerate(SAMPLE_RATE/4)
                    wave_file.writeframes(data.tobytes())
                    return
            elif outputChoice == "csv":

                with open("output.csv", mode='w', newline='') as file:
                    writer = csv.writer(file)
                    for value in data:
                        writer.writerow([value])

                print(f"CSV file saved to output.csv")
                return
            
            elif outputChoice == "png":
                plt.figure(figsize=(10, 4))
                plt.plot(data, linewidth=0.5)
                plt.title("Audio Waveform")
                plt.xlabel("Sample")
                plt.ylabel("Amplitude")
                # plt.ylim(0,65535)
                plt.tight_layout()
                plt.savefig("output.png")
                plt.close()
                print(f"PNG file saved to output.png")
                return

            else:
                print("invalid input")
                continue

   








if __name__ == '__main__':
    menu()