# This file is the optimised, same logic version of Camera.py
import sensor
import time
from pyb import UART

widSize = 480
robot = False
draw = True

CENTER_X = widSize // 2 + 3
CENTER_Y = widSize // 2 - 15
MAX_RADIUS = 200
MIN_RADIUS = 45
INNER_CX = CENTER_X
INNER_CY = CENTER_Y - 50
OUTER_CY = CENTER_Y - 30
MIN_RADIUS_SQ = MIN_RADIUS * MIN_RADIUS
MAX_RADIUS_SQ = MAX_RADIUS * MAX_RADIUS

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((widSize, widSize))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=22.0)
sensor.set_auto_whitebal(False, rgb_gain_db=(0.0, 0.0, 0.0))
sensor.set_auto_exposure(False, exposure_us=30000)

uart = UART(3, 115200, timeout_char=100)

if robot:
    goal_thresholds = [(0, 38, -128, 11, -128, -16), (0, 100, -128, 127, 12, 127)]
    ball_threshold = [(38, 100, 29, 127, 25, 127)]
else:
    # goal_thresholds = [(30, 100, -128, 35, -128, -18), (0, 100, -128, 127, 12, 127)]
    # ball_threshold = [(0, 100, 30, 127, 36, 127)]
    goal_thresholds = [(), ()]
    ball_threshold = [(0, 100, 25, 127, 0, 127)]

ROI_SIZE = 75
MAX_LOST_FRAMES = 10
lost_ball_count = MAX_LOST_FRAMES
last_ball_x = CENTER_X
last_ball_y = CENTER_Y
data = [[CENTER_X, CENTER_Y], [CENTER_X, CENTER_Y], [CENTER_X, CENTER_Y]]
frame = 0

clock = time.clock()


def in_valid_zone(blob):
    dx = blob.cx() - INNER_CX
    dy = blob.cy() - INNER_CY
    if dx * dx + dy * dy <= MIN_RADIUS_SQ:
        return False
    dx = blob.cx() - CENTER_X
    dy = blob.cy() - OUTER_CY
    return dx * dx + dy * dy < MAX_RADIUS_SQ


while True:
    clock.tick()
    img = sensor.snapshot()
    frame += 1
    data[2][0] = data[2][1] = 488

    if frame % 4 == 0:
        data[0][0], data[0][1] = CENTER_X, CENTER_Y
        data[1][0], data[1][1] = CENTER_X, CENTER_Y

        yellow = blue = None
        for blob in img.find_blobs(goal_thresholds, x_stride=4, y_stride=4,
                                   area_threshold=10, pixel_threshold=200, margin=23):
            if not in_valid_zone(blob):
                continue
            code = blob.code()
            if code == 1 and (yellow is None or blob.area() > yellow.area()):
                yellow = blob
            elif code == 2 and (blue is None or blob.area() > blue.area()):
                blue = blob

        if yellow:
            data[0][0] = widSize - yellow.cx()
            data[0][1] = widSize - yellow.cy()
            if draw:
                img.draw_rectangle(yellow.rect(), color=(0, 0, 255))
        if blue:
            data[1][0] = widSize - blue.cx()
            data[1][1] = widSize - blue.cy()
            if draw:
                img.draw_rectangle(blue.rect(), color=(255, 255, 0))
    elif draw:
        if data[0][0] != CENTER_X:
            img.draw_cross(widSize - data[0][0], widSize - data[0][1], color=(0, 0, 255))
        if data[1][0] != CENTER_X:
            img.draw_cross(widSize - data[1][0], widSize - data[1][1], color=(255, 255, 0))

    roi = (0, 0, widSize, widSize)
    if lost_ball_count < MAX_LOST_FRAMES:
        x = max(0, last_ball_x - ROI_SIZE // 2)
        y = max(0, last_ball_y - ROI_SIZE // 2)
        roi = (x, y, min(ROI_SIZE, widSize - x), min(ROI_SIZE, widSize - y))
        if draw:
            img.draw_rectangle(roi, color=(0, 255, 0))

    blob = None
    for b in img.find_blobs(ball_threshold, roi=roi, x_stride=1, y_stride=1,
                            area_threshold=9, pixel_threshold=9, merge=True, margin=5):
        if in_valid_zone(b) and (blob is None or b.area() > blob.area()):
            blob = b

    if blob:
        data[2][0] = widSize - blob.cx()
        data[2][1] = widSize - blob.cy()
        last_ball_x = blob.cx()
        last_ball_y = blob.cy()
        lost_ball_count = 0
        if draw:
            img.draw_rectangle(blob.rect(), color=(255, 165, 0))
    else:
        lost_ball_count += 1

    if draw:
        img.draw_circle(CENTER_X, OUTER_CY, MAX_RADIUS, color=(255, 255, 255), thickness=2)
        img.draw_circle(INNER_CX, INNER_CY, MIN_RADIUS, color=(255, 0, 0), thickness=2)

    uart.writechar(255)
    uart.writechar(250)
    for x, y in data:
        uart.writechar(x >> 1)
        uart.writechar(y >> 1)
    print(clock.fps())
