# pylint: disable=C0103

import sensor

def initialize_sensor():
    """
    Initialize the sensor with the required settings
    """
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(True)
    sensor.set_hmirror(True)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_brightness(2)
    return sensor
