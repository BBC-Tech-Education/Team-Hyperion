# OpenMV Camera Code with Goal Tracking via Region of Interest (ROI)
import sensor
import time
from pyb import UART

widSize = 480
robot = False # control true, chaos false
draw = False

if robot:
    CENTER_X = widSize // 2 + 5
    CENTER_Y = widSize // 2 - 40
    MAX_RADIUS = 180
    MIN_RADIUS = 47
    INNER_CX = CENTER_X - 3
    INNER_CY = CENTER_Y - 0
else:
    CENTER_X = widSize // 2 + 13 # 18
    CENTER_Y = widSize // 2 - 37
    MAX_RADIUS = 205
    MIN_RADIUS = 60
    INNER_CX = CENTER_X - 3
    INNER_CY = CENTER_Y - 5

MIN_RADIUS_SQ = MIN_RADIUS * MIN_RADIUS
MAX_RADIUS_SQ = MAX_RADIUS * MAX_RADIUS
ORIGIN = widSize // 2

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((widSize, widSize))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=22.0)
sensor.set_auto_whitebal(False, rgb_gain_db=(0.0, 0.0, 0.0))
sensor.set_auto_exposure(False, exposure_us=8000)

uart = UART(3, 115200, timeout_char=100)

# yellow, blue
if robot:
    goal_thresholds = [(0, 100, -128, 127, -128, -11), (29, 100, -11, 22, 24, 127)]
    ball_threshold = [(47, 100, 19, 127, 38, 127)]#[(42, 100, -128, 127, 36, 127)]
else:
    goal_thresholds = [(26, 35, -9, 10, -128, -13), (0, 55, 0, 10, 17, 53)]
    ball_threshold = [(42, 100, 48, 127, 24, 127)]

ROI_SIZE_BALL = 75
ROI_SIZE_GOAL = 120
MAX_LOST_FRAMES = 10

last_ball_x, last_ball_y = CENTER_X, CENTER_Y
lost_ball_count = MAX_LOST_FRAMES

last_yellow_x, last_yellow_y = CENTER_X, CENTER_Y
lost_yellow_count = MAX_LOST_FRAMES

last_blue_x, last_blue_y = CENTER_X, CENTER_Y
lost_blue_count = MAX_LOST_FRAMES

data = [[ORIGIN, ORIGIN], [ORIGIN, ORIGIN], [ORIGIN, ORIGIN]]

clock = time.clock()


def in_valid_zone(blob):
    dx = blob.cx() - INNER_CX
    dy = blob.cy() - INNER_CY
    if dx * dx + dy * dy <= MIN_RADIUS_SQ:
        return False
    dx = blob.cx() - CENTER_X
    dy = blob.cy() - CENTER_Y
    return dx * dx + dy * dy < MAX_RADIUS_SQ


def to_mirror(cx, cy):
    return ORIGIN + CENTER_X - cx, ORIGIN + CENTER_Y - cy


def get_roi(last_x, last_y, roi_size, lost_count):
    if lost_count < MAX_LOST_FRAMES:
        x = max(0, last_x - roi_size // 2)
        y = max(0, last_y - roi_size // 2)
        return (x, y, min(roi_size, widSize - x), min(roi_size, widSize - y))
    return (0, 0, widSize, widSize)


while True:
    clock.tick()
    img = sensor.snapshot()
    data[2][0] = data[2][1] = 488

    yellow_roi = get_roi(last_yellow_x, last_yellow_y, ROI_SIZE_GOAL, lost_yellow_count)
    blue_roi = get_roi(last_blue_x, last_blue_y, ROI_SIZE_GOAL, lost_blue_count)

    yellow = blue = None

    for blob in img.find_blobs([goal_thresholds[0]], roi=yellow_roi, x_stride=4, y_stride=4,
                               area_threshold=150, pixels_threshold=200, margin=23):
        if in_valid_zone(blob) and (yellow is None or blob.area() > yellow.area()):
            yellow = blob

    for blob in img.find_blobs([goal_thresholds[1]], roi=blue_roi, x_stride=4, y_stride=4,
                               area_threshold=150, pixels_threshold=200, margin=23):
        if in_valid_zone(blob) and (blue is None or blob.area() > blue.area()):
            blue = blob

    if yellow:
        data[0][0], data[0][1] = to_mirror(yellow.cx(), yellow.cy())
        last_yellow_x, last_yellow_y = yellow.cx(), yellow.cy()
        lost_yellow_count = 0
        if draw:
            img.draw_rectangle(yellow.rect(), color=(0, 0, 255))
    else:
        lost_yellow_count += 1
        data[0] = [ORIGIN, ORIGIN]

    if blue:
        data[1][0], data[1][1] = to_mirror(blue.cx(), blue.cy())
        last_blue_x, last_blue_y = blue.cx(), blue.cy()
        lost_blue_count = 0
        if draw:
            img.draw_rectangle(blue.rect(), color=(255, 255, 0))
    else:
        lost_blue_count += 1
        data[1] = [ORIGIN, ORIGIN]

    ball_roi = get_roi(last_ball_x, last_ball_y, ROI_SIZE_BALL, lost_ball_count)
    if draw and lost_ball_count < MAX_LOST_FRAMES:
        img.draw_rectangle(ball_roi, color=(0, 255, 0))

    blob = None
    for b in img.find_blobs(ball_threshold, roi=ball_roi, x_stride=1, y_stride=1,
                            area_threshold=0, pixels_threshold=0, merge=True, margin=1):
        if in_valid_zone(b) and (blob is None or b.area() > blob.area()):
            blob = b

    if blob:
        data[2][0], data[2][1] = to_mirror(blob.cx(), blob.cy())
        last_ball_x, last_ball_y = blob.cx(), blob.cy()
        lost_ball_count = 0
        if draw:
            img.draw_rectangle(blob.rect(), color=(255, 165, 0))
            img.draw_line(CENTER_X, CENTER_Y, blob.cx(), blob.cy(), color=(255, 165, 0), thickness=2)
    else:
        lost_ball_count += 1

    if draw:
        img.draw_circle(CENTER_X, CENTER_Y, MAX_RADIUS, color=(255, 255, 255), thickness=2)
        img.draw_circle(INNER_CX, INNER_CY, MIN_RADIUS, color=(255, 0, 0), thickness=2)

    packet = bytearray([255, 250])
    for x, y in data:
        safe_x = max(0, min(255, int(x) >> 1))
        safe_y = max(0, min(255, int(y) >> 1))
        packet.append(safe_x)
        packet.append(safe_y)
    uart.write(packet)
    # print(data[2][1]-240)
