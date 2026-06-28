import serial
import time
import sys
import select

# Pi5 UART test script for Teensy SerialPassthroughDemo
ser = serial.Serial(
    port='/dev/ttyAMA0',        # Pi5 GPIO UART (pins 8 TX / 10 RX)
    baudrate=115200,
    timeout=0.1
)

print("=== Pi5 <-> Teensy UART Test ===")
print("Type messages and press Enter.")
print("Ctrl+C to quit.\n")

try:
    while True:
        # Read from Teensy
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if line:
                print(f"Teensy → {line}")

        # Non-blocking keyboard input
        if sys.stdin in select.select([sys.stdin], [], [], 0.01)[0]:
            user_input = input()
            if user_input.strip():
                ser.write((user_input + '\n').encode('utf-8'))
                print(f"Sent → {user_input}")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()