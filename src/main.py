# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor, image, time, math
from pyb import UART
threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(0, 69, -118, -25, 7, 82)]
thresholds1 = [(21, 69, 27, 113, -13, 127)]
uart = UART(3, 115200)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()
blob=0
# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
cx=0
flag=0
while(True):
    flag=0
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs(thresholds, roi=(0, 65, 160, 76), pixels_threshold=100, area_threshold=100, merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255,0,0))
            img.draw_line(blob.major_axis_line(), color=(0,255,0))
            img.draw_line(blob.minor_axis_line(), color=(0,0,255))
            # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
        if blob.cx()>99:
            uart.write("g"+str(blob.cx())+"\n")
        if blob.cx()<100 and blob.cx()>9:
            uart.write("g0"+str(blob.cx())+"\n")
        if blob.cx()<10:
            uart.write("g00"+str(blob.cx())+"\n")


        print(blob.cx())

        flag=1




    for blob in img.find_blobs(thresholds1, roi=(0, 65, 160, 76), pixels_threshold=100, area_threshold=100, merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.5:
            img.draw_edges(blob.min_corners(), color=(255,0,0))
            img.draw_line(blob.major_axis_line(), color=(0,255,0))
            img.draw_line(blob.minor_axis_line(), color=(0,0,255))
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

        if blob.cx()>99:
            uart.write("r"+str(blob.cx())+"\n")
        if blob.cx()<100 and blob.cx()>9:
            uart.write("r0"+str(blob.cx())+"\n")
        if blob.cx()<10:
            uart.write("r00"+str(blob.cx())+"\n")
        print(blob.cx())

        flag=1




    if flag==0:
        uart.write("-1\n")
        print("999")


    #print(clock.fps())


