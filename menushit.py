
import numpy as np
import wave
import serial
import serial.tools.list_ports

BAUD_RATE = 115200
SAMPLE_RATE = 10000

def manual_trigger():
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
            data.append(int.from_bytes(ch))


        data = np.array(data)

        data = (data - data.min()) / data.max()
        data = data * 255

        data = data.astype(np.uint8)

        with wave.open("output.wav", 'wb') as wave_file:    
            wave_file.setnchannels(1)
            wave_file.setsampwidth(1)
            wave_file.setframerate(SAMPLE_RATE)
            wave_file.writeframes(data.tobytes())
        
    except KeyboardInterrupt:
        menu()
    
def distance_trigger():
    print("You are in Distance Trigger Mode: Press ctrl + C for menu\n")
    try:
        while True:
            pass 

    except KeyboardInterrupt:
        menu()
    
def menu():
    """
    Function to run basic menu functionality
    """
    try:    
        while True:
            decision = input("\nYou are in the menu:\nPlease choose a mode by entering a number:\nManual triggering: 1\nDistance Trigger Mode: 2\nOr enter ctrl + C to exit.\n")
            if decision == "1":
                manual_trigger()
                break
            elif decision == "2":
                distance_trigger()
                break
            elif decision == "0":
                exit()
            else:
                print("invalid input, try again\n")
                continue
    except KeyboardInterrupt:
        print("\nExiting program")
        exit()







if __name__ == '__main__':
    menu()