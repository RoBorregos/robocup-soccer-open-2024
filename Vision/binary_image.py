import sensor
import time
from pyb import UART

uart = UART(3, 115200, timeout_char=0)
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.skip_frames(time=2000)
clock = time.clock()


red_threshold = (86, 100, -128, 127, -128, 127)
FRAME_HEIGHT = 160
FRAME_WIDTH = 120
FRAME_ROBOT = 15
LINE_THRESHOLD = 60

while True:
    for i in range(100):
        clock.tick()
        img = sensor.snapshot()
        img.binary([red_threshold])

        count_white_front = 0
        count_white_back = 0
        count_white_left = 0
        count_white_right = 0

        #Front pixels
        for i in range(130, 140):
            for j in range(100, 150):
                if img.get_pixel(i, j) == (255, 255, 255):

                    count_white_front += 1

        for i in range(130, 150):
            for j in range(90, 100):
                if img.get_pixel(i, j) == (255, 255, 255):
                    count_white_left += 1

        for i in range(130, 180):
            for j in range(150, 160):
                if img.get_pixel(i, j) == (255, 255, 255):
                    count_white_right += 1

        for i in range(180, 190):
            for j in range(100, 150):
                if img.get_pixel(i, j) == (255, 255, 255):
                    count_white_back += 1

        line_flag_back = False
        line_flag_left = False
        line_flag_right = False
        line_flag_front = False

        if count_white_back > LINE_THRESHOLD:
            line_flag_back = True
            uart.write('b')
            print(count_white_back)
            print("move back")
        elif count_white_left > LINE_THRESHOLD:
            line_flag_left = True
            uart.write('l')
            print(count_white_left)
            print("move left")
        elif count_white_right > LINE_THRESHOLD:
            uart.write('r')
            line_flag_right = True
            print(count_white_right)
            print("move right")
        elif count_white_front > LINE_THRESHOLD:
            uart.write('f')
            line_flag_front = True
            print(count_white_front)
            print("move front")
        else:
            uart.write('n')

        img.draw_rectangle(130, 100, 10, 50, color=(0, 255, 0))
        img.draw_rectangle(130, 90, 50, 10, color=(0, 255, 0))
        img.draw_rectangle(130, 150, 50, 10, color=(0, 255, 0))
        img.draw_rectangle(180, 100, 10, 50, color=(0, 255, 0))
        img.draw_circle(FRAME_HEIGHT+3, FRAME_WIDTH-5, FRAME_ROBOT, color=0x0000, fill=True)

