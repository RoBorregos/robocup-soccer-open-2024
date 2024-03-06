# pylint: disable=C0103

"""
Method to calculate the distance of a blob from the hiperbolic mirror camera
"""
import math

FRAME_HEIGHT = 160
FRAME_WIDTH = 120
PIXEL_SIZE_HEIGHT = 0.2726875
PIXEL_SIZE_WIDTH = 0.25

def calculate_distance(blob):
    """Calculate the distance of the blob from the camera"""
    relative_cx = blob.cx() - FRAME_HEIGHT
    relative_cy = blob.cy() - FRAME_WIDTH

    cm_cx = relative_cx * PIXEL_SIZE_HEIGHT
    cm_cy = relative_cy * PIXEL_SIZE_WIDTH
    distance = math.sqrt(cm_cx*2 + cm_cy*2)
    distance *= 0.1

    camera_height = 5
    floor_height = 13
    tangent = (1/2)*(0.9*distance)*(0.045+0.45*distance*2)*(-1/2)
    tangent_angle = math.degrees(math.atan(tangent))
    y = ((0.1+distance*2)*0.45)*(1/2)+camera_height
    center_angle = math.degrees(math.atan((y/distance)))
    projection_angle = 2*(tangent_angle)-(center_angle)
    total_distance = distance+(floor_height/(math.tan(math.radians(-projection_angle))))

    return total_distance
