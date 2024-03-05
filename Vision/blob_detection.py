

# Import module for board related functions
# Import the module for sensor related functions
# Import module containing machine vision algorithms
# Import module for tracking elapsed time
import math, utime, time, image, sensor, pyb
from pyb import UART

uart = UART(3, 9600) # Initiates the UART communication with a baudrate of 9600

# Define the min/max LAB values we're looking for
thresholdsCube = (62, 79, 22, 127, 6, 127)

sensor.reset() # Resets the sensor
sensor.set_pixformat(sensor.RGB565) # Sets the sensor to RGB
sensor.set_framesize(sensor.QVGA) # Sets the resolution to 320x240 px
sensor.set_vflip(True) # Flips the image vertically
sensor.set_hmirror(True) # Mirrors the image horizontally
sensor.skip_frames(time = 2000) # Skip some frames to let the image stabilize
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking



ledRed = pyb.LED(1) # Initiates the red led
ledGreen = pyb.LED(2) # Initiates the green led

clock = time.clock() # Instantiates a clock object
sensor.set_brightness(2) # Sets the brightness of the image to 2

while(True):
    clock.tick() # Advances the clock
    img = sensor.snapshot() # Takes a snapshot and saves it in memory

    # Find blobs with a minimal area of 50x50 = 2500 px
    # Overlapping blobs will be merged
    blobs = img.find_blobs([thresholdsCube, thresholdsCube] , area_threshold=1, merge=True)


    # Draw blobs
    for blob in blobs:
        # Draw a rectangle where the blob was found
        img.draw_rectangle(blob.rect(), color=(0,255,0))
        # Draw a cross in the middle of the blob
        img.draw_cross(blob.cx(), blob.cy(), color=(0,255,0))
        uart.write("Blob found at x: " + str(blob.cx()) + " y: " + str(blob.cy()) + "\n") # Sends the x and y coordinates of the blob to the serial console
        #(Find relative cx and cy
        rcx = blob.cx() - 50 # 50 is camera res
        rcy = blob.cy() - 100 # 100 is camera res
        print(rcx, rcy)
        # Find distance using relative coordinates
        distance = math.sqrt(rcx**2 + rcy**2)
        print(distance)
        # Find angle but first check sign 
        if rcx < 180:
            angle = math.atan(rcy/rcx) + 180
        else:
            angle = math.atan(rcy/rcx)
        print(angle)

    pyb.delay(50) # Pauses the execution for 50ms
    print(clock.fps()) # Prints the framerate to the serial console
    print(blob.cx(), blob.cy()) # Prints the x and y coordinates of the blob to the serial console
    print(blob.w()) # Prints the width of the blob to the serial console
    print(blob.h()) # Prints the height of the blob to the serial console
    uart.wrrite(blob.w())
    
