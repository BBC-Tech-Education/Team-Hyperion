import sensor
import time
from pyb import UART

# --- Configuration & Settings ---
exposure = 3000
widSize = 480
robot = False # Chaos = False, Control = True
draw = True

# --- Useable Zone Settings ---
CENTER_X = int(widSize / 2) + 3
CENTER_Y = int(widSize / 2) - 15
MAX_RADIUS = 200
MIN_RADIUS = 45
INNER_OFFSET_X = -0
INNER_OFFSET_Y = -50
OUTER_OFFSET_Y = -30
INNER_CX = CENTER_X + INNER_OFFSET_X
INNER_CY = CENTER_Y + INNER_OFFSET_Y
OUTER_CY = CENTER_Y + OUTER_OFFSET_Y
MIN_RADIUS_SQ = MIN_RADIUS ** 2
MAX_RADIUS_SQ = MAX_RADIUS ** 2

# --- Camera Settings ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((widSize, widSize))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=22.0)
sensor.set_auto_whitebal(False, rgb_gain_db=(0.0, 0.0, 0.0))
sensor.set_auto_exposure(False, exposure_us=30000)

# --- UART Settings ---
clock = time.clock()
uart = UART(3, 115200, timeout_char=100)

# --- Frame Buffer Settings ---
ROI_SIZE = 75
MAX_LOST_FRAMES = 10
last_ball_x = CENTER_X
last_ball_y = CENTER_Y
lost_ball_count = MAX_LOST_FRAMES

# --- Threshold Values ---
if robot:
    goal_thresholds = [(0, 38, -128, 11, -128, -16), (0, 100, -128, 127, 12, 127)]
    ball_threshold = [(38, 100, 29, 127, 25, 127)]
else:
    goal_thresholds = [(30, 100, -128, 35, -128, -18), (0, 100, -128, 127, 12, 127)]
    ball_threshold = [(0, 100, 30, 127, 36, 127)]


# Data array persists across frames now.
# Index 0: Goal 1, Index 1: Goal 2, Index 2: Ball
data = [[CENTER_X, CENTER_Y], [CENTER_X, CENTER_Y], [CENTER_X, CENTER_Y]]

# --- Goal Scan Delay ---
frame_counter = 0  # Frame counter for time-slicing
GOAL_UPDATE_RATE = 4  # Delay between frames to scan for goals


# Data array persists across frames now.
# Index 0: Goal 1, Index 1: Goal 2, Index 2: Ball
data = [[CENTER_X, CENTER_Y], [CENTER_X, CENTER_Y], [CENTER_X, CENTER_Y]]

# --- Goal Scan Delay ---
frame_counter = 0  # Frame counter for time-slicing
GOAL_UPDATE_RATE = 4  # Delay between frames to scan for goals


def in_valid_zone(blob):
    '''Determinds if a blob is within the valid bounds'''
    inner_dist_sq = (blob.cx() - INNER_CX)**2 + (blob.cy() - INNER_CY)**2
    if inner_dist_sq <= MIN_RADIUS_SQ:
        return False
    outer_dist_sq = (blob.cx() - CENTER_X)**2 + (blob.cy() - OUTER_CY)**2
    return outer_dist_sq < MAX_RADIUS_SQ


while True:
    clock.tick()
    img = sensor.snapshot()
    frame_counter += 1

    # We always reset the ball data so we don't send ghost ball data if it's lost
    data[2][0], data[2][1] = 488, 488

    # --- GOAL TRACKING (Time-Sliced) ---
    if (frame_counter % GOAL_UPDATE_RATE == 0):
        # Reset goal data only on the frames we actually scan for them
        data[0][0], data[0][1] = CENTER_X, CENTER_Y
        data[1][0], data[1][1] = CENTER_X, CENTER_Y

        # Stride increased to 4 for massive performance gain on background scans
        goal_blobs = img.find_blobs(goal_thresholds, x_stride=4, y_stride=4, area_threshold=10,
                                    pixel_threshold=200, merge=False, margin=23)
        goal_blobs = sorted(goal_blobs, key=lambda blob: -blob.area())

        for blob in goal_blobs:
            if in_valid_zone(blob):
                if (blob.code() == 1) and (data[0] == [CENTER_X, CENTER_Y]):
                    data[0][0] = widSize - blob.cx()
                    data[0][1] = widSize - blob.cy()
                    if draw:
                        img.draw_rectangle(blob.rect(), color=(0, 0, 255))
                elif (blob.code() == 2) and (data[1] == [CENTER_X, CENTER_Y]):
                    data[1][0] = widSize - blob.cx()
                    data[1][1] = widSize - blob.cy()
                    if draw:
                        img.draw_rectangle(blob.rect(), color=(255, 255, 0))
            else:
                if draw:
                    img.draw_rectangle(blob.rect(), color=(255, 0, 0))
                    img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))
    else:
        # If we skipped scanning this frame, optionally draw crosses at the cached goal locations
        if draw:
            if data[0] != [CENTER_X, CENTER_Y]:
                img.draw_cross(widSize - data[0][0], widSize - data[0][1], color=(0, 0, 255))
            if data[1] != [CENTER_X, CENTER_Y]:
                img.draw_cross(widSize - data[1][0], widSize - data[1][1], color=(255, 255, 0))

    current_roi = (0, 0, widSize, widSize)
    if lost_ball_count < MAX_LOST_FRAMES:
        roi_x = max(0, last_ball_x - (ROI_SIZE // 2))
        roi_y = max(0, last_ball_y - (ROI_SIZE // 2))
        roi_w = min(ROI_SIZE, widSize - roi_x)
        roi_h = min(ROI_SIZE, widSize - roi_y)
        current_roi = (roi_x, roi_y, roi_w, roi_h)
        if draw:
            img.draw_rectangle(current_roi, color=(0, 255, 0))

    # Ball stride remains at 1 because it's a small object inside a small ROI
    ball_blobs = img.find_blobs(ball_threshold, roi=current_roi, x_stride=1, y_stride=1, area_threshold=9,
                                pixel_threshold=9, merge=True, margin=5)
    ball_blobs = sorted(ball_blobs, key=lambda blob: -blob.area())

    ball_found_this_frame = False

    for blob in ball_blobs:
        if in_valid_zone(blob):
            if data[2] == [488, 488]:
                data[2][0] = widSize - blob.cx()
                data[2][1] = widSize - blob.cy()
                last_ball_x = blob.cx()
                last_ball_y = blob.cy()
                ball_found_this_frame = True

                if draw:
                    img.draw_rectangle(blob.rect(), color=(255, 165, 0))
                    img.draw_line(INNER_CX, INNER_CY, blob.cx(), blob.cy(), color=(255, 165, 0), thickness=2)
                break
        else:
            if draw:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0))
                img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 0))

    if not ball_found_this_frame:
        lost_ball_count += 1
    else:
        lost_ball_count = 0

    if draw:
        img.draw_circle(CENTER_X, OUTER_CY, MAX_RADIUS, color=(255, 255, 255), thickness=2)
        img.draw_circle(INNER_CX, INNER_CY, MIN_RADIUS, color=(255, 0, 0), thickness=2)

    uart.writechar(255)
    uart.writechar(250)
    for item in data:
        uart.writechar(item[0] >> 1)
        uart.writechar(item[1] >> 1)

    print(item)
