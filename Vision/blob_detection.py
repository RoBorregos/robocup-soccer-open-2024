'''
This script is used to detect a blob in the image and send the x and y 
coordinates of the blob to the serial console, calculate distance and angle.
'''

from pyb import UART
import time
from relative_distance import calculate_distance
from initialize_sensor import initialize_sensor
from find_blob import find_blobs, draw_blobs
from find_angle import calculate_angle

uart = UART(3, 9600)
threshold = (62, 79, 22, 127, 6, 127)
sensor = initialize_sensor()
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = find_blobs(img, threshold)
    draw_blobs(img, blobs)

    for blob in blobs:
        total_distance = calculate_distance(blob)
        print(f"Distance: {total_distance}")
        angle = calculate_angle(blob)
        print(f"Angle: {angle}")
    pyb.delay(50)
