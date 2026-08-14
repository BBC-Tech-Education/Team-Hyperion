import csi
import time
from machine import UART, LED

# SETTINGS
CALIBRATION = False

# exposure = 13000
widSize = 240
robot = False # control = true, chaos = false

csi0 = csi.CSI()

# Camera Set Up
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.window((widSize,widSize))
w#csi0.windowing((widSize, widSize))

csi0.snapshot(time=2000)

clock = time.clock()

# UART
uart = UART(3, 115200, timeout_char=100)

# Thresholds | blue : yellow : Orange |
if(robot):
    t = [(0, 35, -128, 127, -128, -22),(0, 60, 14, 33, 26, 127), (0, 0, -128, -128, -128, -128)]
else:
    t = [(0, 35, -128, 127, -128, -22),(0, 100, 30, 127, 39, 127), (0, 0, -128, -128, -128, -128)]

if CALIBRATION:
    print("Calib")

else:
    csi0.auto_gain(False, gain_db=19.82785)
    csi0.auto_exposure(False, exposure_us=16648)
    csi0.auto_whitebal(False)

if CALIBRATION:
    while(True):
        img = csi0.snapshot()
        print("Gain:", csi0.gain_db(), "Exposure:", csi0.exposure_us())
else:
    while True:

        # Camera Data
        img = csi0.snapshot()
        blobs = sorted(img.find_blobs(t,  pixels_threshold=20, area_threshold=20, merge=True), key=lambda blob: blob.area())
        data = [[245,245],[245,245],[245,245]]
        for blob in blobs:
            if(blob.code() < 3):
                data[blob.code()] = [blob.cx(),blob.cy()]
                img.draw_rectangle(blob.x(),blob.y(),blob.w(),blob.h())


        # Sending Data
        uart.writechar(255)
        uart.writechar(250)
        uart.writechar(data[2][0])
        uart.writechar(data[2][1])
        uart.writechar(data[1][0])
        uart.writechar(data[1][1])
        uart.writechar(data[0][0])
        uart.writechar(data[0][1])
        # uart.writechar(data[0][2]) <-- Add in later
        print(data)
