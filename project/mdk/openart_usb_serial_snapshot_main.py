# OpenART Plus USB serial snapshot receiver.
# Save/burn this file as main.py on the OpenART drive.
#
# PC sends:
#   s : save one photo
#   h : print help
#   p : ping
#
# Photos are saved as /cmd_0001.jpg, /cmd_0002.jpg ...

import sensor
import time
import os
import pyb


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)

usb = pyb.USB_VCP()


def next_photo_name():
    index = 1
    while True:
        name = "/cmd_%04d.jpg" % index
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


def print_help():
    print("commands: s=save photo, p=ping, h=help")


print("usb serial snapshot ready")
print_help()

last_print_ms = time.ticks_ms()

while True:
    sensor.snapshot()

    if usb.any():
        data = usb.read()
        if data:
            for ch in data:
                if ch == ord("s") or ch == ord("S"):
                    save_photo()
                elif ch == ord("p") or ch == ord("P"):
                    print("pong")
                elif ch == ord("h") or ch == ord("H") or ch == ord("?"):
                    print_help()

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print_ms) >= 5000:
        print("running, send s to save")
        last_print_ms = now
