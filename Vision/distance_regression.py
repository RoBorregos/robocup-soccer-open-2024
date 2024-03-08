# Import module for board related functions
# Import the module for sensor related functions
# Import module containing machine vision algorithms
# Import module for tracking elapsed time
import math, utime, time, image, sensor, pyb
from pyb import UART

uart = UART(3, 9600) # Initiates the UART communication with a baudrate of 9600

# Define the min/max LAB values we're looking for
thresholdsCube = (52, 85, 22, 97, 8, 61)

sensor.reset() # Resets the sensor
sensor.set_pixformat(sensor.RGB565) # Sets the sensor to RGB
sensor.set_framesize(sensor.QVGA) # Sets the resolution to 320x240 px
sensor.set_vflip(True) # Flips the image vertically
sensor.set_hmirror(True) # Mirrors the image horizontally
sensor.skip_frames(time = 2000) # Skip some frames to let the image stabilize
#sensor.set_auto_gain(False) # must be turned off for color tracking
#sensor.set_auto_whitebal(False) # must be turned off for color tracking



#ledRed = pyb.LED(1) # Initiates the red led
#ledGreen = pyb.LED(2) # Initiates the green led

clock = time.clock() # Instantiates a clock object
#sensor.set_brightness(2) # Sets the brightness of the image to 2

while(True):
    clock.tick() # Advances the clock
    img = sensor.snapshot() # Takes a snapshot and saves it in memory

    # Find blobs with a minimal area of 50x50 = 2500 px
    # Overlapping blobs will be merged
    blobs = img.find_blobs([thresholdsCube, thresholdsCube] , area_threshold=1, merge=True)
    img.draw_cross(160, 120, color=(30,255,10), size = 200)

    # Draw blobs
    for blob in blobs:
        # Draw a rectangle where the blob was found
        #img.draw_circle(blob.rect(), color=(0,255,0))
        # Draw a cross in the middle of the blob
        img.draw_cross(blob.cx(), blob.cy(), color=(0,255,0))
        #uart.write("Blob found at x: " + str(blob.cx()) + " y: " + str(blob.cy()) + "\n") # Sends the x and y coordinates of the blob to the serial console
        #(Find relative cx and cy
        rcx = blob.cx() - 160 # 50 is camera res
        rcy = blob.cy() - 120 # 100 is camera res
        distance_pixel = math.sqrt(rcx**2 + rcy**2)
        angulo = math.atan2(rcy, rcx)
        degreess = math.degrees(angulo)
        print(f"angulo{degreess}")
        #print(blob.cx())
        #print(distance_pixel)
        #print(rcx)
        #print(rcy)
        #print(rcx, rcy)
        # Find distance using relative coordinates
        #mcx = rcx * 0.2375
        #mcy = rcy * 0.2375

        #print(mcx)
        cirle_radius=blob.enclosing_circle()[2]
        #distance = math.sqrt(mcx**2 + mcy**2)
        #distance=distance-circleradius
        '''distance *= 0.1
        #print("dist:  ",distance)
        #print(f"distance{distance}")

        # Conversion distancia relativa a distancia real
        #---------------------------------------------------------------#
        hcam = 5
        hfloor = 15
        tangente = (1/2)*(0.9*distance)*(0.045+0.45*distance**2)**(-1/2)
        anguloTan = math.degrees(math.atan(tangente))
        y = ((0.1+distance**2)*0.45)**(1/2)+hcam
        anguloCentro = math.degrees(math.atan((y/distance)))
        Y=2*(180-90-(anguloCentro-anguloTan))
        X=(180-Y)/2
        anguloProject =X-anguloTan
        distTotal = (distance+(hfloor/(math.tan(math.radians(anguloProject)))))

        #print(distTotal)




        #----------------------------------------------------------------#


    pyb.delay(50) # Pauses the execution for 50ms
    #print(clock.fps()) # Prints the framerate to the serial console
    #print(blob.cx(), blob.cy()) # Prints the x and y coordinates of the blob to the serial console
    #print(blob.w()) # Prints the width of the blob to the serial console
    #print(blob.h()) # Prints the height of the blob to the serial console
    #uart.write(blob.w())'''
        distTotal= 11.83*math.exp((0.0245)*distance_pixel)
        #print(distTotal)
    pyb.delay(50)
