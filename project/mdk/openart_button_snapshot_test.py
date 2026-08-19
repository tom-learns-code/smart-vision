# OpenART Plus / OpenMV IDE burn test.
# Function: press the board button to save a photo as /snap_0001.jpg, /snap_0002.jpg...
#
# Usage:
# 1. Run it in OpenMV IDE first.
# 2. If the serial terminal prints "button source: ...", press the board key.
# 3. After saving photos, reset/replug the board so Windows refreshes the U disk.

import sensor
import time
import os


# ---------------- Camera setup ----------------

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240, good for snapshot test.
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)


# ---------------- Button adapter ----------------

class ButtonReader:
    def __init__(self):
        self.name = None
        self.read = None
        self.last_pressed = False

        self._try_pyb_switch()
        if self.read is None:
            self._try_machine_pins()

    def _try_pyb_switch(self):
        try:
            import pyb
            sw = pyb.Switch()

            # Some boards create the object but fail only when read.
            sw.value()

            self.name = "pyb.Switch()"
            self.read = lambda: bool(sw.value())
        except Exception:
            self.name = None
            self.read = None

    def _try_machine_pins(self):
        try:
            from machine import Pin
        except Exception:
            return

        # Common user-key names used by different OpenMV/OpenART firmwares.
        candidates = (
            "KEY", "KEY0", "KEY1", "KEY2",
            "USER", "USER_KEY", "USER_BTN",
            "BUTTON", "BUTTON1", "SW", "SW1",
            "WAKEUP", "WKUP",
            "P0", "P1", "P2", "P3", "P4", "P5",
        )

        for pin_name in candidates:
            try:
                pin = Pin(pin_name, Pin.IN, Pin.PULL_UP)
                pin.value()
                self.name = "machine.Pin('%s', PULL_UP), active low" % pin_name
                self.read = lambda pin=pin: (pin.value() == 0)
                return
            except Exception:
                pass

        for pin_name in candidates:
            try:
                pin = Pin(pin_name, Pin.IN, Pin.PULL_DOWN)
                pin.value()
                self.name = "machine.Pin('%s', PULL_DOWN), active high" % pin_name
                self.read = lambda pin=pin: (pin.value() == 1)
                return
            except Exception:
                pass

    def pressed_edge(self):
        if self.read is None:
            return False

        pressed = bool(self.read())
        edge = pressed and (not self.last_pressed)
        self.last_pressed = pressed
        return edge


def next_photo_name():
    index = 1
    while True:
        name = "/snap_%04d.jpg" % index
        try:
            os.stat(name)
            index += 1
        except Exception:
            return name


button = ButtonReader()

print("button snapshot test ready")
if button.read is None:
    print("button source: NONE")
    print("If the board key does nothing, tell me this line.")
else:
    print("button source:", button.name)


# ---------------- Main loop ----------------

last_print_ms = time.ticks_ms()

while True:
    img = sensor.snapshot()

    if button.pressed_edge():
        filename = next_photo_name()
        img.save(filename, quality=90)
        print("saved:", filename)
        time.sleep_ms(500)     # simple debounce and avoid repeated photos

    # Heartbeat for OpenMV IDE serial terminal.
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print_ms) > 2000:
        print("running")
        last_print_ms = now
