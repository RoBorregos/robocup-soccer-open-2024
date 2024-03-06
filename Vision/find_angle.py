# pylint: disable=C0103

'''
Calculates the angle of the blob from the center of the camera

'''
import math

FRAME_HEIGHT = 160
FRAME_WIDTH = 120

def calculate_angle(blob):
    """Calculate the angle of the blob from the center of the camera"""
    rcx = blob.cx() - FRAME_HEIGHT
    rcy = blob.cy() - FRAME_WIDTH

    if rcx == 0 or rcy == 0:
        return None
    if rcx < 180 and rcx != 0:
        angle = math.atan(rcy/rcx) + 180
    else:
        angle = math.atan(rcy/rcx)
    return angle
