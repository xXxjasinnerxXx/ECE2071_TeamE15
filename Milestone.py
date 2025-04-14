import serial
import serial.tools.list_ports

word = bytes("word", "utf-8")  # Word to send to STM32
DELAY_BETWEEN_REQUESTS = 0.1
TIMEOUT = 1  # Timeout set to 1 second

# List available serial devices
devices = serial.tools.list_ports.comports()
# for device in devices:
#     print(device.device)

# Open the serial port (update this with your correct port)
ser = serial.Serial("/dev/cu.usbmodem103", 115200, timeout=TIMEOUT)  # Replace with your port
print("Connected to STM32")

# Send the word to STM32
print("Sending word...")
ser.write(word)
print("Sent word")

# Wait for and read the response from STM32
response = ser.read(6)  # Adjust to read exactly what you expect, i.e., "ACK\r\n" or "NOPE\r\n"
print("yuhhhh")
if response:
    decoded = response.decode().strip()  # Strip any newline or extra spaces
    print(f"Received: {decoded}")
else:
    print("No response received")

ser.close()