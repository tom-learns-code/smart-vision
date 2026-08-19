import gc
import sensor
import time
import tf
from machine import UART
from machine import Pin


FP_BUILD_ID = 7200120
ROI = (55, 40, 245, 180)
FP_UART_ID = 12
UART_BAUD = 115200
MODEL_PATHS = (
    "/sd/smartcar_class_model_qat_1.tflite",
    "/sd/smartcar_class_model_qat_2.tflite",
)
LABEL_PATH = "/sd/model_labels.txt"

_CMM_LINES = (
    "hw,-,-,rt117x,seekfree_art_plus,",
    "uart,12,TXD,-,LPSR_06,",
    "uart,12,RXD,-,LPSR_07,",
)


def load_cmm_config():
    import cmm

    pin_map = {}
    for line in _CMM_LINES:
        parts = line.split(",")
        if parts[1] == "-":
            name = parts[0] + "." + parts[2]
        else:
            name = parts[0] + "." + parts[1] + "." + parts[2]
        try:
            pin = Pin(parts[4])
        except Exception:
            pin = None
        pin_map[name] = (parts[3], parts[4], pin, None)
    cmm.add(pin_map)


def mem_free():
    try:
        return gc.mem_free()
    except Exception:
        return -1


def load_labels(path):
    labels = []
    with open(path, "r") as label_file:
        for line in label_file:
            label = line.strip()
            if label:
                labels.append(label)
    return labels


def highest_score(scores):
    best_id = 0
    best_confidence = scores[0]
    for class_id in range(1, len(scores)):
        if scores[class_id] > best_confidence:
            best_id = class_id
            best_confidence = scores[class_id]
    return best_id, best_confidence


def release_fb_model(model_name):
    try:
        tf.free_from_fb()
        print(
            "FP_DUAL_STAGE model=%s phase=FREE_FB_OK mem=%d"
            % (model_name, mem_free())
        )
    except Exception as error:
        print(
            "FP_DUAL_STAGE model=%s phase=FREE_FB_FAIL mem=%d err=%s"
            % (model_name, mem_free(), error)
        )


def run_one_model(model_index, labels):
    model_name = "image%d" % (model_index + 1)
    model_path = MODEL_PATHS[model_index]
    gc.collect()
    started = time.ticks_ms()
    print(
        "FP_DUAL_STAGE model=%s phase=LOAD_BEGIN mem=%d"
        % (model_name, mem_free())
    )

    net = tf.load(model_path, load_to_fb=True)
    load_ms = time.ticks_diff(time.ticks_ms(), started)
    print(
        "FP_DUAL_STAGE model=%s phase=CAPTURE_BEGIN load=%dms mem=%d"
        % (model_name, load_ms, mem_free())
    )

    capture_started = time.ticks_ms()
    img = sensor.snapshot()
    capture_ms = time.ticks_diff(time.ticks_ms(), capture_started)
    print(
        "FP_DUAL_STAGE model=%s phase=INFER_BEGIN capture=%dms mem=%d"
        % (model_name, capture_ms, mem_free())
    )

    infer_started = time.ticks_ms()
    best_id = 0xFF
    best_confidence = 0.0
    for result in tf.classify(
        net,
        img,
        min_scale=1.0,
        scale_mul=0.5,
        x_overlap=0.0,
        y_overlap=0.0,
    ):
        scores = result.output()
        if scores:
            class_id, confidence = highest_score(scores)
            if best_id == 0xFF or confidence > best_confidence:
                best_id = class_id
                best_confidence = confidence
    infer_ms = time.ticks_diff(time.ticks_ms(), infer_started)

    label = "?"
    if best_id != 0xFF and best_id < len(labels):
        label = labels[best_id]
    print(
        "FP_DUAL_RESULT model=%s id=%d label=%s conf=%.4f load=%dms capture=%dms infer=%dms mem=%d"
        % (
            model_name,
            best_id,
            label,
            best_confidence,
            load_ms,
            capture_ms,
            infer_ms,
            mem_free(),
        )
    )

    del img
    gc.collect()
    release_fb_model(model_name)
    del net
    gc.collect()


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(ROI)
sensor.set_framerate(60)
sensor.set_auto_exposure(False, exposure_us=80)
sensor.skip_frames(time=2000)

load_cmm_config()
fp_uart = UART(FP_UART_ID, UART_BAUD)
gc.collect()

labels = load_labels(LABEL_PATH)
print("========== FP2 Compact CMM/UART Diagnostic =============")
print(
    "FP_BUILD id=%d mode=COMPACT_CMM_UART_DUAL models=2 uart=%d labels=%d mem=%d"
    % (FP_BUILD_ID, FP_UART_ID, len(labels), mem_free())
)
print("ROI: x=%d y=%d w=%d h=%d" % ROI)
print("=======================================================")

cycle = 0
while True:
    cycle += 1
    print("FP_DUAL_CYCLE_BEGIN cycle=%d mem=%d" % (cycle, mem_free()))
    run_one_model(0, labels)
    time.sleep_ms(100)
    run_one_model(1, labels)
    print("FP_DUAL_CYCLE_END cycle=%d mem=%d" % (cycle, mem_free()))
    time.sleep_ms(500)
