# OpenART Plus burn test: auto snapshot.
# Burn/save this file as main.py. After power-up it saves photos automatically.
#
# It does not need board keys or keyboard input.
# It saves /burn_0001.jpg, /burn_0002.jpg ... then keeps running.

import sensor
import time
import os


PHOTO_INTERVAL_MS = 5000
PHOTO_MAX_COUNT = 5


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)


def next_photo_name():
    index = 1
    while True:
        name = "/burn_%04d.jpg" % index
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


print("auto snapshot burn test ready")
print("It will save one photo every 5 seconds, max 5 photos.")

last_photo_ms = time.ticks_ms()
last_print_ms = time.ticks_ms()
photo_count = 0

while True:
    sensor.snapshot()
    now = time.ticks_ms()

    if photo_count < PHOTO_MAX_COUNT:
        if time.ticks_diff(now, last_photo_ms) >= PHOTO_INTERVAL_MS:
            save_photo()
            photo_count += 1
            last_photo_ms = now

    if time.ticks_diff(now, last_print_ms) >= 2000:
        print("running, photos:", photo_count)
        last_print_ms = now
