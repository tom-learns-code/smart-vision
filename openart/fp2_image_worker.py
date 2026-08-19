import gc
import machine
import os
import sensor
import time
import tf
import cmm
from machine import Pin
from machine import UART


FP_BUILD_ID = 7200137
JOB = "/sd/fp2_job.bin"
STATE = "/sd/fp2_state.bin"
LAST = "/sd/fp2_last.bin"
TRACE = "/sd/fp2_trace.bin"
MODELS = ("/sd/smartcar_class_model_qat_1.tflite",
          "/sd/smartcar_class_model_qat_2.tflite")
ROI = (55, 40, 245, 180)
THRESHOLD = 0.80
SAMPLE_COUNT = 3


def xor(data, length=None):
    value = 0
    if length is None:
        length = len(data)
    for index in range(length):
        value ^= data[index]
    return value


def read(path):
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except Exception:
        return None


def write(path, data):
    with open(path, "wb") as handle:
        handle.write(data)


def trace(stage):
    try:
        write(TRACE, bytes((stage,)))
        try:
            os.sync()
        except Exception:
            pass
    except Exception:
        pass


def restart_dispatcher():
    try:
        machine.soft_reset()
    except Exception:
        pass
    try:
        machine.reset()
    except Exception:
        pass
    try:
        machine.WDT(timeout=1000)
        while True:
            time.sleep_ms(100)
    except Exception:
        raise SystemExit


def valid(data, frame_type):
    return (data is not None and len(data) == 20
            and data[0] == 0xA5 and data[1] == 0x5A
            and data[2] == 0xF2 and data[3] == 1
            and data[4] == frame_type and xor(data, 19) == data[19])


def make_uart():
    pins = {
        "hw.-": ("rt117x", "seekfree_art_plus", None, None),
        "uart.12.TXD": ("-", "LPSR_06", Pin("LPSR_06"), None),
        "uart.12.RXD": ("-", "LPSR_07", Pin("LPSR_07"), None),
    }
    cmm.add(pins)
    del pins
    gc.collect()
    return UART(12, 115200)


def make_probe():
    packet = bytearray((0xA5, 0x5A, 0x31, 12, 1,
                        FP_BUILD_ID & 0xFF,
                        (FP_BUILD_ID >> 8) & 0xFF,
                        (FP_BUILD_ID >> 16) & 0xFF,
                        (FP_BUILD_ID >> 24) & 0xFF, 0))
    packet[9] = xor(packet, 9)
    return packet


def make_result(job, result_id, confidence, status):
    packet = bytearray((
        0xA5, 0x5A, 0xF2, 1, 0x44, job[5], job[6], job[7], job[8],
        result_id, confidence, status, 1,
        FP_BUILD_ID & 0xFF, (FP_BUILD_ID >> 8) & 0xFF,
        (FP_BUILD_ID >> 16) & 0xFF, (FP_BUILD_ID >> 24) & 0xFF,
        0, 0, 0,
    ))
    packet[19] = xor(packet, 19)
    return packet


def state_valid(state, session):
    return (state is not None and len(state) == 13
            and state[0] == 0xF2 and state[1] == 1
            and state[2] == session and state[5] <= 3
            and xor(state, 12) == state[12])


def compact_id(raw_id):
    state = bytearray(read(STATE) or b"")
    if len(state) != 13 or xor(state, 12) != state[12]:
        return 0xFF, False
    count = state[5]
    for index in range(count):
        offset = 6 + index * 2
        if state[offset] == raw_id:
            return state[offset + 1], True
    capacity = max(state[3], state[4])
    if count >= capacity or count >= 3:
        return 0xFF, False
    offset = 6 + count * 2
    state[offset] = raw_id
    state[offset + 1] = count
    state[5] = count + 1
    state[12] = xor(state, 12)
    write(STATE, state)
    return count, True


def highest(scores):
    best_id = 0
    best_confidence = scores[0]
    for class_id in range(1, len(scores)):
        if scores[class_id] > best_confidence:
            best_id = class_id
            best_confidence = scores[class_id]
    return best_id, best_confidence


def classify(path, stage):
    net = None
    trace(stage)
    try:
        gc.collect()
        net = tf.load(path, load_to_fb=True)
        score_sums = [0.0] * 10
        valid_samples = 0
        for sample in range(SAMPLE_COUNT):
            img = sensor.snapshot()
            for result in tf.classify(net, img, min_scale=1.0, scale_mul=0.5,
                                      x_overlap=0.0, y_overlap=0.0):
                scores = result.output()
                if scores:
                    for class_id in range(len(scores)):
                        score_sums[class_id] += scores[class_id]
                    valid_samples += 1
                    break
            del img
        if valid_samples == 0:
            return 0xFF, 0.0, 7
        best_id, confidence_sum = highest(score_sums)
        return best_id, confidence_sum / valid_samples, 0
    except Exception:
        return 0xFF, 0.0, 7
    finally:
        if net is not None:
            try:
                tf.free_from_fb()
            except Exception:
                pass
            del net
        gc.collect()


job = read(JOB)
try:
    os.remove(JOB)
except Exception:
    pass
trace(1)
if not valid(job, 0x42):
    trace(0xE1)
    restart_dispatcher()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(ROI)
sensor.set_framerate(60)
sensor.set_auto_exposure(False, exposure_us=80)
sensor.set_vflip(True)
sensor.skip_frames(time=500)
trace(2)

uart = make_uart()
uart.write(make_probe())
trace(3)
state = read(STATE)
raw_id = 0xFF
result_id = 0xFF
confidence = 0.0
status = 0

if not state_valid(state, job[5]):
    status = 3
else:
    first = classify(MODELS[0], 4)
    trace(5)
    second = classify(MODELS[1], 6)
    trace(7)
    first_ok = first[2] == 0 and first[1] >= THRESHOLD
    second_ok = second[2] == 0 and second[1] >= THRESHOLD
    if first_ok or second_ok:
        chosen = first if first_ok and (not second_ok or first[1] >= second[1]) else second
        raw_id, confidence, status = chosen
    elif first[0] != 0xFF or second[0] != 0xFF:
        chosen = first if first[1] >= second[1] else second
        raw_id, confidence, status = chosen[0], chosen[1], 1
    else:
        status = 2
    if status == 0:
        result_id, mapped = compact_id(raw_id)
        if not mapped:
            status = 7

percent = int(confidence * 100.0 + 0.5)
percent = max(0, min(100, percent))
packet = make_result(job, result_id, percent, status)
write(LAST, packet)
try:
    os.sync()
except Exception:
    pass
trace(8)
uart.write(packet)
time.sleep_ms(300)
restart_dispatcher()
