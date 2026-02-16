import serial
import time

# Open Pi UART
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

time.sleep(2)  # Let connection settle

print("Sending PING...")
ser.write(b'PING\n')

time.sleep(1)

if ser.in_waiting:
    response = ser.readline().decode().strip()
    print("Received from Teensy:", response)
else:
    print("No response received.")

ser.close()