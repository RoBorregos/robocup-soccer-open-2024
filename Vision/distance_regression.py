"""
This script makes a regression between pixels and real distance measures in cm and angles

The script uses the camera to detect a colored object and calculate the distance from the camera to 
the object. The distance of the blob from the camera is calculated using an exponential regression 
model. The model is based on the magnitude of the distance of the blob from the center of the image. 
Angle is calculated using atan2 function.

Author: Jocelyn Velarde
Version: March 7, 2024
"""
import math
import time
import utime
import image
import sensor
import pyb
from pyb import UART

uart = UART(3, 9600, timeout_char=0)
thresholds = (52, 85, 22, 97, 8, 61)

FRAME_HEIGHT = 160
FRAME_WIDTH = 120
PIXEL_SIZE_HEIGHT = 0.2726875
PIXEL_SIZE_WIDTH = 0.25
CAMERA_HEIGHT = 5
FLOOR_HEIGHT = 15
CM_CONVERTION = 0.1

def initialize_sensor():
    """
    Initializes the camera sensor with the required settings.
    """
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(True)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=2000)
    sensor.set_brightness(-1)
    sensor.set_contrast(-2)
    sensor.set_saturation(1)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)


def locate_blob(img):
    """
    Locates the blob in the image and returns the blob object.
    """
    blobs = img.find_blobs([thresholds, thresholds],
                           area_threshold=1, merge=True)
    img.draw_cross(FRAME_HEIGHT, FRAME_WIDTH, color=(30, 255, 10), size=200)

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
    return blobs

def calculate_distance(blob):
    """
    Calculates the distance of the blob from the camera.
    """
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    magnitude_distance = math.sqrt(relative_cx**2 + relative_cy**2)
    # Exponential regression model calculated using real data points with pixel comparision
    total_distance = 11.83*math.exp((0.0245)*magnitude_distance)
    return total_distance

def calculate_angle(blob):
    """
    Calculates the angle of the blob from the camera.
    """
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    angle = math.degrees(math.atan(relative_cy/relative_cx))
    return angle + 180 if (angle < 0) else angle

def main():
    """
    Main function of the script.
    """
    initialize_sensor()
    clock = time.clock()
    while True:
        clock.tick()
        img = sensor.snapshot()
        blobs = locate_blob(img)
        if blobs:
            distance = calculate_distance(blobs[0])
            angle = calculate_angle(blobs[0])
            print("Distance: ", distance, " cm")
            print("Angle: ", angle, " degrees")
            uart.write("{:.2f} {:.2f}\n".format(distance, angle))
        else:
            print("No blob found")
            uart.write("No blob found\n")
        pyb.delay(50)

if __name__ == "__main__":
    main()
