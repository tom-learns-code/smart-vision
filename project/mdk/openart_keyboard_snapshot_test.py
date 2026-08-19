# OpenART Plus / OpenMV IDE keyboard snapshot test.
# Click the serial terminal, type "s", then press Enter to save a photo.
# Type "h" then Enter to show help.
#
# This version uses sys.stdin so it matches the OpenMV IDE serial terminal.

import sensor
import time
import os
import sys
import select


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


def next_photo_name():
    index = 1
    while True:
        name = "/snap_%04d.jpg" % index
        try:
            os.stat(name)
            index += 1
        except Exception:
            return name


def save_photo():
    img = sensor.snapshot()
    filename = next_photo_name()
    img.save(filename, quality=90)
    print("saved:", filename)


print("keyboard snapshot test ready")
print("Click serial terminal, type s, then press Enter to save a photo.")
print("Type h then Enter for help.")

last_print_ms = time.ticks_ms()

while True:
    sensor.snapshot()

    if poll.poll(0):
        line = sys.stdin.readline()
        if line:
            line = line.strip()
            if line == "s" or line == "S":
                save_photo()
            elif line == "h" or line == "H" or line == "?":
                print("commands: s=save photo, h=help")
            else:
                print("unknown command:", line)

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print_ms) > 3000:
        print("running, click serial terminal and type s then Enter")
        last_print_ms = now
