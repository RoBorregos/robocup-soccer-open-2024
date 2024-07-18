
#defiendo porteria amarilla y no me meto a la azul
import math
import time
import utime
import image
import sensor
import pyb
from pyb import UART

uart = UART(3, 115200, timeout_char=0)
thresholds_blob1 =  (26, 77, 51, 76, 27, 55)    #(100, 72, 4, 127, 0, 94)   #(22, 81, 50, 75, -25, 74) (22, 81, 50, 70, -10, 24)
thresholds_blob2 = (52, 93, -6, 77, 33, 127)      #(80, 89, -18, 6, 19, 127)
thresholds_azul = (17, 36, 0, 54, -48, -10)
thresholds_line = (86, 100, -128, 127, -128, 127)

FRAME_HEIGHT = 160
FRAME_WIDTH = 120
FRAME_ROBOT = 23
FRAME_CIRCLE = 150
PIXEL_SIZE_HEIGHT = 0.2726875
PIXEL_SIZE_WIDTH = 0.25
CAMERA_HEIGHT = 5
FLOOR_HEIGHT = 15
CM_CONVERTION = 0.1
minor_x = 0
minor_y = 0

distance_ball = 0
distance_pixels = 0
angle_ball = 0
distance_goal = 0
angle_goal = 0
final_goal_distance = 0



def initialize_sensor():
    """
    Initializes the camera sensor with the required settings.
    """
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(False)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=2000)
    sensor.set_brightness(-4)
    sensor.set_contrast(2)
    sensor.set_saturation(5)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)


def locate_blob(img):
    """
    Locates the blobs in the image for each threshold set and returns a list of blob objects for each set.
    """
    img.draw_circle(FRAME_HEIGHT+3, FRAME_WIDTH-6, FRAME_ROBOT, color=0x0000, fill=True)
    img.draw_circle(FRAME_HEIGHT+3, FRAME_WIDTH-5, FRAME_CIRCLE, color=0x0000, fill=False, thickness=100)
    blobs1 = img.find_blobs([thresholds_blob1], area_threshold=1, merge=True)
    blobs2 = img.find_blobs([thresholds_blob2], area_threshold=150, merge=True)
    blobAzul = img.find_blobs([thresholds_azul], area_threshold=150, merge=True)
    #blobs3 = img.find_blobs([thresholds_line], area_threshold=1, merge=True)


    for blob in blobs1:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))

    for blob in blobs2:
        #print("amarillo")
        img.draw_ellipse(blob.enclosed_ellipse(), color=(255, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

    for blob in blobAzul:
        print("azul")
        img.draw_ellipse(blob.enclosed_ellipse(), color=(0, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

    #for blob in blobs3:
        #img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        #img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))

    return blobs1, blobs2, blobAzul

def calculate_distance(blob):
    """
    Calculates the distance of the blob from the camera (hypotenuse).
    """
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    magnitude_distance = math.sqrt(relative_cx**2 + relative_cy**2)
    #print(magnitude_distance)
    # Exponential regression model calculated using real data points with pixel comparison
    total_distance = 9.2021 **(magnitude_distance * 0.0199)
    print(total_distance)
    return total_distance

def calculate_opp_distance(goal_distance, goal_angle):
    """
    Calculates the distance of the blob from the camera (opposite side). Considering angle variation.
    """
    if(goal_angle > 270):
        goal_angle = 360 - goal_angle
    distance_final = goal_distance*math.sin (math.radians(goal_angle))
    return distance_final

def calculate_distance_goal(blob):
    """
    Calculates the distance of the blob from the camera.
    """
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    magnitude_distance = math.sqrt(relative_cx**2 + relative_cy**2)
    #print(magnitude_distance)
    # Exponential regression model calculated using real data points with pixel comparison
    total_distance = 1.2375*magnitude_distance-18.162
    return total_distance



def calculate_angle(blob):
    """
    Calculates the angle of the blob from the camera.
    """
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH
    angle = math.atan2(relative_cy, relative_cx)
    angle_degrees = math.degrees(angle)

    angle_degrees += 95
    # Adjust the angle to the range of 0 to 360 degrees
    if angle_degrees < 0:
        angle_degrees += 360
    return angle_degrees

def main():
    """
    Main function of the script.
    """
    initialize_sensor()
    clock = time.clock()
    global distance_ball, angle_ball,angle_goal, distance_pixels

    while True:


        distance_ball = 0
        angle_ball = 0
        distance_goal = 0
        angle_goal = 0
        final_goal_distance = 0
        clock.tick()
        img = sensor.snapshot()
        blobs1, blobs2, blobAzul = locate_blob(img)

        if blobs1 :
            for blob in blobs1:
                distance_ball = calculate_distance(blob)
                angle_ball = calculate_angle(blob)
                #print("Blob 1 - Distance: ", distance_ball, " cm")
                #print("Blob 1 - Angle: ", angle_ball, " degrees")
                #uart.write("{:.2f} {:.2f}\n".format(distance, angle))

        if blobs2:
            for blob in blobs2:
                #distance_pixels = calculate_distance(blob)
                #distance_pixels = calculate_distance_goal(blob)
                angle_goal = calculate_angle(blob)
                #final_goal_distance = calculate_opp_distance(distance_goal, angle_goal)
                #print("Blob 2 - Distance: ", distance, " cm")
                #print("Blob 2 - Angle: ", angle, " degrees")
                #uart.write("{:.2f} {:.2f}\n".format(distance, angle))
        if blobAzul:
            for blob in blobAzul:
                distance_pixels = calculate_distance_goal(blob)
                angle_goal = calculate_angle(blob)


                #print("Blob A - Distance: ", final_goal_distance, " cm")
        #print(" Distance ball: ", distance_ball, "cm\n", "Angle ball: ", angle_ball, " degrees\n", "Distance Goal: ", final_goal_distance, "cm\n", "Angle Goal: ", angle_goal, " degrees\n", "Pre Distance Goal: ", distance_goal)
        #print("Distance Goal: ", final_goal_distance, "cm\n", "Angle Goal: ", angle_goal, " degrees\n")
        #uart.write("{} {} {} {}\n".format(int(distance_ball), int(angle_ball), int(angle_goal)));
        uart.write("{:.2f} {:.2f} {:.2f} {:.2f}\n". format(distance_ball, angle_ball, angle_goal, distance_pixels))
        print(angle_goal)
        #print(distance_pixels)
        #uart.write(f"{distance_ball},{angle_ball},{angle_goal},{distance_goal}\n")
        pyb.delay(50)

if __name__ == "__main__":
    main()


