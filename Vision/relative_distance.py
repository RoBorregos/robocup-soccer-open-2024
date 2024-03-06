# pylint: disable=C0103

"""
Method to calculate the distance of a blob from the hiperbolic mirror camera
"""
import math

frame_height = 160
frame_width = 120
pixel_size_height = 0.2726875
pixel_size_width = 0.25

def calculate_distance(blob):
    """Calculate the distance of the blob from the camera"""
    relative_cx = blob.cx() - frame_height
    relative_cy = blob.cy() - frame_width

    cm_cx = relative_cx * pixel_size_height
    cm_cy = relative_cy * pixel_size_width
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
