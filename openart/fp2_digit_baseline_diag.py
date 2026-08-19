import os
import sensor
import image
import time
import tf
from machine import UART


# Camera crop copied from roi_capture(2).py.
# Coordinates are based on a 320 x 240 QVGA frame.
ROI_X = 55
ROI_Y = 40
ROI_W = 245
ROI_H = 180
ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

# Put these two files in the root directory of the SD card.
# Relative-path fallbacks are included for firmware that starts in /sd.
MODEL_CANDIDATES = ("/sd/smartcar_digit_model_qat.tflite", "smartcar_digit_model_qat.tflite")
LABEL_CANDIDATES = (
    "/sd/digit_labels.txt",
    "digit_labels.txt",
)

# A result below this value is displayed as "unknown".
CONFIDENCE_THRESHOLD = 0.80

# Temporary UART-port probe. Every opened camera UART broadcasts a packet that
# contains its own UART ID. The MCU reports the ID seen on UART1/B12/B13.
FP_BUILD_ID = 7200114
UART_BAUD = 115200
UART_CANDIDATES = (5, 12, 11)
FP_PROBE_PERIOD_MS = 2000
FP_HEADER_0 = 0xA5
FP_HEADER_1 = 0x5A
FP_TYPE_PORT_PROBE = 0x31

_CMM_LINES = (
    "hw,-,-,rt117x,seekfree_art_plus,",
    "uart,5,TXD,-,AD_28,",
    "uart,5,RXD,-,AD_29,",
    "uart,11,TXD,-,LPSR_04,",
    "uart,11,RXD,-,LPSR_05,",
    "uart,12,TXD,-,LPSR_06,",
    "uart,12,RXD,-,LPSR_07,",
)


def load_cmm_config():
    from machine import Pin
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


def append_u32_le(packet, value):
    packet.append(value & 0xFF)
    packet.append((value >> 8) & 0xFF)
    packet.append((value >> 16) & 0xFF)
    packet.append((value >> 24) & 0xFF)


def xor_bytes(data):
    value = 0
    for item in data:
        value ^= item
    return value


def make_probe_packet(uart_id, sequence):
    packet = bytearray((
        FP_HEADER_0,
        FP_HEADER_1,
        FP_TYPE_PORT_PROBE,
        uart_id,
        sequence,
    ))
    append_u32_le(packet, FP_BUILD_ID)
    packet.append(xor_bytes(packet))
    return packet


def send_uart_probes():
    global probe_sequence
    global last_probe_tick

    now = time.ticks_ms()
    if last_probe_tick is not None and time.ticks_diff(now, last_probe_tick) < FP_PROBE_PERIOD_MS:
        return

    last_probe_tick = now
    probe_sequence = (probe_sequence + 1) & 0xFF
    sent_ids = []
    for uart_id, uart in probe_uarts:
        try:
            uart.write(make_probe_packet(uart_id, probe_sequence))
            sent_ids.append(str(uart_id))
        except Exception as error:
            print("FP UART%d write failed: %s" % (uart_id, error))
    print(
        "FP_PROBE fp_id=%d seq=%d uart_ids=%s"
        % (FP_BUILD_ID, probe_sequence, ",".join(sent_ids))
    )


def find_existing_file(candidates):
    for path in candidates:
        try:
            os.stat(path)
            return path
        except:
            pass
    raise OSError("file not found: %s" % (candidates,))


def load_labels(path):
    result = []
    with open(path, "r") as label_file:
        for line in label_file:
            label = line.strip()
            if label:
                result.append(label)
    return result


def highest_score(scores):
    best_index = 0
    best_score = scores[0]
    for index in range(1, len(scores)):
        if scores[index] > best_score:
            best_index = index
            best_score = scores[index]
    return best_index, best_score


# Keep the ROI preview script's camera initialization unchanged.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(ROI)
sensor.set_framerate(60)
sensor.set_auto_exposure(False, exposure_us=80)
sensor.skip_frames(time=2000)

clock = time.clock()

model_path = find_existing_file(MODEL_CANDIDATES)
label_path = find_existing_file(LABEL_CANDIDATES)
labels = load_labels(label_path)
if not labels:
    raise ValueError("label file is empty")

print("Loading model: %s" % model_path)
net = tf.load(model_path, load_to_fb=True)

try:
    load_cmm_config()
    print("FP cmm.add OK")
except Exception as error:
    print("FP cmm.add failed: %s" % error)

probe_uarts = []
for probe_uart_id in UART_CANDIDATES:
    try:
        probe_uart = UART(probe_uart_id, UART_BAUD)
        probe_uarts.append((probe_uart_id, probe_uart))
        print("FP UART%d OK" % probe_uart_id)
    except Exception as error:
        print("FP UART%d unavailable: %s" % (probe_uart_id, error))

if not probe_uarts:
    raise OSError("no probe UART opened")

probe_sequence = 0
last_probe_tick = None

print("========== ROI Classifier Ready ==========")
print("FP_BUILD id=%d mode=DIGIT_BASELINE load_to_fb=1 probe=UART5/12/11 period=%dms" % (FP_BUILD_ID, FP_PROBE_PERIOD_MS))
print("ROI: x=%d y=%d w=%d h=%d" % ROI)
print("Labels: %d" % len(labels))
print("==========================================")

output_size_checked = False

while True:
    send_uart_probes()
    clock.tick()
    img = sensor.snapshot()

    # The sensor has already cropped the frame to ROI. Run one classification
    # window over that complete ROI; tf.classify resizes it to the model input.
    for result in tf.classify(
        net,
        img,
        min_scale=1.0,
        scale_mul=0.5,
        x_overlap=0.0,
        y_overlap=0.0,
    ):
        scores = result.output()
        if not scores:
            continue

        if not output_size_checked:
            if len(scores) != len(labels):
                print(
                    "WARNING: model outputs %d classes, label file has %d"
                    % (len(scores), len(labels))
                )
            output_size_checked = True

        class_index, confidence = highest_score(scores)
        if class_index < len(labels):
            predicted_label = labels[class_index]
        else:
            predicted_label = "class_%d" % class_index

        if confidence >= CONFIDENCE_THRESHOLD:
            display_label = predicted_label
            color = (0, 255, 0)
        else:
            display_label = "unknown"
            color = (255, 0, 0)

        img.draw_rectangle(result.rect(), color=color)
        img.draw_string(
            2,
            2,
            "%s %.1f%%" % (display_label, confidence * 100.0),
            color=color,
        )

        print(
            "label=%s index=%d confidence=%.4f fps=%.2f"
            % (display_label, class_index, confidence, clock.fps())
        )
