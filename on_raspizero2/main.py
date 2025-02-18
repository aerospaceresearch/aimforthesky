import time
import serial
import adafruit_bno055
import argparse
import geomag
import sys
import board
import digitalio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import json


class Sensor:
    def __init__(self):
        self.azimuth = 0.0
        self.elevation = 0.0
        self.roll = 0.0
        
        # geo position, "null island"
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        
        # gnss
        self.nextupdate = 0.0
        self.nextdt = 1.0 # period for allowing updates
        self.fix = 0
        
        # magnetic correction to true north
        self.declination = 0.0
        
        # UART setup for BNO055 using PySerial
        self.uart = serial.Serial("/dev/serial0", baudrate=115200)
        self.sensor = adafruit_bno055.BNO055_UART(self.uart)
        
        # OLED display setup
        oled_reset = digitalio.DigitalInOut(board.D4)
        self.WIDTH = 128
        self.HEIGHT = 32
        i2c = board.I2C()
        self.oled = adafruit_ssd1306.SSD1306_I2C(self.WIDTH, self.HEIGHT, i2c, addr=0x3D, reset=oled_reset)
        
        self.oled.fill(0)
        self.oled.show()
        
        self.image = Image.new("1", (self.oled.width, self.oled.height))
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()


        
    def get_position_and_time(self, file_path = "position_time.json"):
        
        if time.time() > self.nextupdate:
            
            self.nextupdate = time.time() + self.nextdt
                
            try:
                # Attempt to open and read the JSON file
                with open(file_path, "r") as file:
                    data = json.load(file)

                # Extract position data
                self.lat = data["position"]["lat"]
                self.lon = data["position"]["lon"]
                self.alt = data["position"]["alt"]

                # Extract time data
                timestamp = data["time"]["timestamp"]
                self.fix = data["time"]["fix"]


            except FileNotFoundError:
                print(f"Error: The file '{file_path}' was not found.")
            except json.JSONDecodeError:
                print(f"Error: The file '{file_path}' is not a valid JSON file.")
            except PermissionError:
                print(f"Error: Permission denied when accessing '{file_path}'.")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")


    def update_sensor_values(self):
        
        # I will change this later to take the value whenever the geo position is called
        self.declination = Sensor.correct_azimuth(self.lat, self.lon)
    
        # Get orientation (azimuth, pitch, roll)
        euler = self.sensor.euler
        if euler is not None:
            self.azimuth, self.elevation, self.roll = euler


    def display_text(self):
        self.draw.rectangle((0, 0, self.WIDTH, self.HEIGHT), outline=0, fill=0)  # Clear screen
        

        fix_sign = "." if self.fix == 0 else "x"
        el_sign = "+" if self.elevation >= 0 else "-"
        self.draw.text((1, 10), f"AZ {abs(self.azimuth):06.2f} EL {el_sign}{abs(self.elevation):05.2f}", font=self.font, fill=255)
        self.draw.text((4, 0), f"O -12.0 C -12.0 t{fix_sign}", font=self.font, fill=255) # Sun and Moon positions
        self.draw.text((1, 19), "* STARNAME", font=self.font, fill=255) # the closest star's name
        
        self.draw.line([(0, 9), (12, 9)], width=1, fill=255)  # horizon line for the Sun. Will make it move according to where the Sun is
        self.draw.line([(50, 9), (59, 9)], width=1, fill=255)  # horizon line for the Moon. Will make it move according to where the Moon is
        
        
        # rudimentary targetting arrows        
        target_el = 45
        target_az = 45
        
        arrow = [(103, 25), (107, 25), (107, 27), (103, 27)]
        
        if abs(self.azimuth - target_az) >= abs(self.elevation - target_el):
            if self.azimuth - target_az > 0:
                arrow = [(100, 23), (110, 25), (100, 27)]
            else:
                arrow = [(110, 23), (100, 25), (110, 27)]
        else:
            if self.elevation - target_el > 0:
                arrow = [(102, 27), (105, 23), (109, 27)]
            else:
                arrow = [(102, 23), (105, 27), (109, 23)]
            
        self.draw.polygon(arrow, fill=255)  # Arrowhead
        
        self.oled.image(self.image)
        self.oled.show()


    @staticmethod
    def correct_azimuth(latitude, longitude):
        """
        Corrects the magnetic azimuth to true north by considering magnetic declination.
    
        :param latitude: Latitude of the measurement location
        :param longitude: Longitude of the measurement location
        :return: Declination (degrees)
        """
        # Get the magnetic declination at the given location
        return geomag.declination(latitude, longitude)


    def run(self):
        try:
            print("Starting sensor display...")
            
            while True:
                self.get_position_and_time()
                self.update_sensor_values()            
                sys.stdout.write(f"\rAz. (Yaw): {self.azimuth:+06.2f}° Decl. {self.declination:.2f}°, Elev. (Pitch): {self.elevation:+05.2f}°, Roll: {self.roll:.2f}°   ")
                sys.stdout.flush()
                self.display_text()
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nExiting program.")
        finally:
            GPIO.cleanup()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Correct azimuth from magnetometer to true north.")
    parser.add_argument("latitude", type=float, help="Latitude of the measurement location")
    parser.add_argument("longitude", type=float, help="Longitude of the measurement location")
    args = parser.parse_args()
    
    sensor = Sensor()
    sensor.lat = args.latitude
    sensor.lon = args.longitude
    sensor.run()
