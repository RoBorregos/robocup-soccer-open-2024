"""
This script is used to detect the distance of a color blob from the camera.

The script uses the camera sensor to capture an image and then locates the
blob in the image. The distance of the blob from the camera is then calculated
using a mathematical model.

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

uart = UART(3, 9600)
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

    cm_cx = relative_cx * PIXEL_SIZE_HEIGHT
    cm_cy = relative_cy * PIXEL_SIZE_WIDTH
    distance = math.sqrt(cm_cx**2 + cm_cy**2)
    distance *= CM_CONVERTION

    tangent = (1/2)*(0.9*distance)*(0.045+0.45*distance**2)**(-1/2)
    tangent_angle = math.degrees(math.atan(tangent))
    y = ((0.1+distance**2)*0.45)**(1/2)+CAMERA_HEIGHT
    center_angle = math.degrees(math.atan((y/distance)))
    off_angle = 2*(180-90-(center_angle-tangent_angle))
    mirror_angle = (180-off_angle)/2
    projection_angle = mirror_angle-tangent_angle
    total_distance = distance + \
        (FLOOR_HEIGHT/(math.tan(math.radians(projection_angle))))
    return total_distance


def main():
    """
    Main function to run the script.
    """
    initialize_sensor()
    clock = time.clock()
    while True:
        clock.tick()
        img = sensor.snapshot()
        blobs = locate_blob(img)

        for blob in blobs:
            total_distance = calculate_distance(blob)
            print("Distance: ", total_distance)
        pyb.delay(50)


if __name__ == "__main__":
    main()
