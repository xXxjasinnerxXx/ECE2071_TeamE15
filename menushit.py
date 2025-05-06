
import numpy as np
import wave
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import os
import csv

BAUD_RATE = 115200
SAMPLE_RATE = 10000
data = []
def manual_trigger(data = None):
    print("You are in Manual trigger mode: Press ctrl + C for menu\n")
    
    try:
        # Select the first com port
        ports = serial.tools.list_ports.comports()
        device = None
        for i in ports:
            if i.description[0:3] == "STM":
                device = i

        print(f"STM Serial Port = {device.device}")

        ser = serial.Serial(device.device, BAUD_RATE)

        data = []
        while True:
            try:
                snippetLength = int(input("Enter length of snippet in seconds: "))
                break
            except ValueError:
                print("Invalid value, try again")
                continue
                
        for i in range(SAMPLE_RATE*snippetLength):
            ch = ser.read(1)
            data.append(int.from_bytes(ch,byteorder="big",signed=True))

        # data = np.array(data, dtype=np.uint16)
        
        data = np.array(data)
        data = (data - data.min()) / data.max()
        data = data * 255

        data = data.astype(np.uint8)
        return data
        
    except KeyboardInterrupt:
        return None
    
def distance_trigger():
    print("You are in Distance Trigger Mode: Press ctrl + C for menu\n")
    try:
        while True:
            pass 

    except KeyboardInterrupt:
        return None
    
def menu():
    """
    Function to run basic menu functionality
    """
    data = None
    try:    
        while True:
            decision = input("\nYou are in the menu:\nPlease choose a mode by entering a number:\nManual triggering: 1\nDistance Trigger Mode: 2\nData Anlysis: 3\nOr enter ctrl + C to exit.\n")
            if decision == "1":
                data = manual_trigger()
            elif decision == "2":
                data = distance_trigger()
            elif decision == "0":
                exit()
            elif decision == "3":
                output_type(data)
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
                with wave.open("output.wav", 'wb') as wave_file:    
                    wave_file.setnchannels(1)
                    wave_file.setsampwidth(1)
                    wave_file.setframerate(SAMPLE_RATE)
                    wave_file.writeframes(data.tobytes())
                    return
            elif outputChoice == "csv":
                desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
                file_path = os.path.join(desktop_path, "output.csv")

                with open(file_path, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    for value in data:
                        writer.writerow([value])

                print(f"CSV file saved to {file_path}")
                return
            
            elif outputChoice == "png":
                plt.figure(figsize=(10, 4))
                plt.plot(data, linewidth=0.5)
                plt.title("Audio Waveform")
                plt.xlabel("Sample")
                plt.ylabel("Amplitude")
                plt.tight_layout()
                desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
                file_path = os.path.join(desktop_path, "output.png")
                plt.savefig(file_path)
                plt.close()
                print(f"PNG file saved to {file_path}")
                return

            else:
                print("invalid input")
                continue

   








if __name__ == '__main__':
    menu()