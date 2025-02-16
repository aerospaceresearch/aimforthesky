import time
import serial
import adafruit_bno055


# UART setup for BNO055 using PySerial
uart = serial.Serial("/dev/serial0", baudrate=115200)  # "/dev/serial0" is the default UART port on Raspberry Pi

# Create the BNO055 sensor object with the PySerial connection
sensor = adafruit_bno055.BNO055_UART(uart)

try:
    print("starting")

    while True:
        
        # Get orientation (azimuth, pitch, roll)
        euler = sensor.euler  # (yaw, pitch, roll)
        if euler is not None:
            yaw, pitch, roll = euler
            # Refresh the output line in terminal
            print(f"\rOrientation - Azimuth (Yaw): {yaw:.2f}°, Elevation (Pitch): {pitch:.2f}°, Roll: {roll:.2f}°", end="")

        time.sleep(0.1)  # Short delay

except KeyboardInterrupt:
    print("\nExiting program.")
finally:
    GPIO.cleanup()  # Reset GPIO settings
