import sensor, image, time, pyb
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()
uart = UART(3, 115200, timeout_char = 10)
thresholds = [(37, 66, -23, 127, -46, -24), (83, 95, -31, -18, -128, 93)] #blue first thresh, then yellow

while(True):
    clock.tick()
    img = sensor.snapshot()
    goalBlobs = img.find_blobs(thresholds, x_stride=5, y_stride=5, area_threshold=50, pixel_threshold=50, merge=False, margin=23)
    blueBlobs = []
    yellowBlobs = []
    data = [254, ]
    if blueBlobs:
        largestBlob = blueBlobs[0]
    if yellowBlobs:
        largestBlob = yellowBlobs[0]
    for blob in goalBlobs:
        if blob.code() == 1:
            blueBlobs.append(blob)
        elif blob.code() == 2:
            yellowBlobs.append(blob)
    if blueBlobs:
        for i in blueBlobs:
            if i.area > largestBlob.area:
                largestBlob = i
    if yellowBlobs:
        for i in yellowBlobs:
                largestBlob = i
#    for i in data:
#        print(largestBlob)
#        print(largestBlob)
#        if i.area > largestBlob.area:
#    if isdebug:
#        print(data)
#        img.draw_cross(int(img.width()/2), int(img.height()/2))

#blob.pixels()
#blob.cx()
#blob.cy()
