import time
import serial
import adafruit_bno055
import argparse
import geomag  # Install with `pip install geomag`
import sys


# UART setup for BNO055 using PySerial
uart = serial.Serial("/dev/serial0", baudrate=115200)  # "/dev/serial0" is the default UART port on Raspberry Pi

# Create the BNO055 sensor object with the PySerial connection
sensor = adafruit_bno055.BNO055_UART(uart)


def correct_azimuth(latitude, longitude):
    """
    Corrects the magnetic azimuth to true north by considering magnetic declination.
    
    :param latitude: Latitude of the measurement location
    :param longitude: Longitude of the measurement location
    :return: True azimuth (degrees)
    """
    # Get the magnetic declination at the given location
    declination = geomag.declination(latitude, longitude)
    
    return declination


def main(declination):
    try:
        print("starting")
    
        while True:
        
            # Get orientation (azimuth, pitch, roll)
            euler = sensor.euler  # (yaw, pitch, roll)
            if euler is not None:
                yaw, pitch, roll = euler
                # Refresh the output line in terminal              
                sys.stdout.write(f"\rAz. (Yaw): {yaw:.2f}° Decl. {declination:.2f}°, Elev. (Pitch): {pitch:.2f}°, Roll: {roll:.2f}°   ")
                sys.stdout.flush()  # Ensure the output is updated immediately

            time.sleep(0.1)  # Short delay

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        GPIO.cleanup()  # Reset GPIO settings


if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Correct azimuth from magnetometer to true north.")
    parser.add_argument("latitude", type=float, help="Latitude of the measurement location")
    parser.add_argument("longitude", type=float, help="Longitude of the measurement location")
    
    # Parse arguments
    args = parser.parse_args()
    
    # Declination for correcting the azimuth
    declination = correct_azimuth(args.latitude, args.longitude)
    
    main(declination)