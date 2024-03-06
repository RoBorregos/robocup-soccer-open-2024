# pylint: disable=C0103

'''
Calculates the angle of the blob from the center of the camera

'''
import math


def calculate_angle(blob):
    """Calculate the angle of the blob from the center of the camera"""
    rcx = blob.cx() - 160
    rcy = blob.cy() - 120

    if rcx == 0 or rcy == 0:
        return None
    if rcx < 180 and rcx != 0:
        angle = math.atan(rcy/rcx) + 180
    else:
        angle = math.atan(rcy/rcx)
    return angle
