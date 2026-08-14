import sensor
import time
from pyb import UART

# --- Configuration & Settings ---
exposure = 30000
widSize = 480
robot = False  # Chaos = False, Control = True
draw = True

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((widSize, widSize))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=22.0)
sensor.set_auto_whitebal(False, rgb_gain_db=(0.0, 0.0, 0.0))
sensor.set_auto_exposure(False, exposure_us=exposure)

clock = time.clock()
uart = UART(3, 115200, timeout_char=100)

if robot:
    goal_thresholds = [(0, 38, -128, 11, -128, -16), (0, 100, -128, 127, 12, 127)]
    ball_threshold = [(46, 100, 30, 127, 37, 127)]
else:
    goal_thresholds = [(30, 100, -128, 35, -128, -18), (0, 100, -128, 127, 12, 127)]
    ball_threshold = [(0, 100, 30, 127, 36, 127)]

ROI_SIZE = 75
MAX_LOST_FRAMES = 10
lost_ball_count = MAX_LOST_FRAMES
last_ball_x = 0
last_ball_y = 0

data = [[0, 0], [0, 0], [488, 488]]

while True:
    clock.tick()
    img = sensor.snapshot()
    current_roi = (0, 0, widSize, widSize)
    if lost_ball_count < MAX_LOST_FRAMES:
        roi_x = max(0, last_ball_x - (ROI_SIZE // 2))
        roi_y = max(0, last_ball_y - (ROI_SIZE // 2))
        roi_w = min(ROI_SIZE, widSize - roi_x)
        roi_h = min(ROI_SIZE, widSize - roi_y)
        current_roi = (roi_x, roi_y, roi_w, roi_h)
        if draw:
            img.draw_rectangle(current_roi, color=(0, 255, 0))
    ball_blobs = img.find_blobs(ball_threshold, x_stride=1, y_stride=1, roi=current_roi, area_threshold=9, pixel_threshold=9, merge=True, margin=5)
    ball_blobs = sorted(ball_blobs, key=lambda blob: -blob.area())
    ball_found_this_frame = False
    for blob in ball_blobs:
        ball_found_this_frame = True
        last_ball_x = blob.cx()
        last_ball_y = blob.cy()
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        data[2][0] = blob.cx()
        data[2][1] = blob.cy()
        # data = [[0, 0], [0, 0], [blob.cx, blob.cy]]

    if not ball_found_this_frame:
        lost_ball_count += 1
    else:
        lost_ball_count = 0

    uart.writechar(255)
    uart.writechar(250)
    for item in data:
        uart.writechar(item[0] >> 1)
        uart.writechar(item[1] >> 1)
    print(data[2])
