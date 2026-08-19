import gc
import sensor
import time
import tf
from machine import Pin
from machine import UART


FP_BUILD_ID = 7200141
UART_ID = 12
UART_BAUD = 115200
ROI = (55, 40, 245, 180)
IMAGE_MODEL = "/sd/smartcar_class_model_qat_1.tflite"
DIGIT_MODEL = "/sd/smartcar_digit_model_qat.tflite"
CLASS_COUNT = 10
SAMPLE_COUNT = 3
THRESHOLD = 0.80
PROBE_MS = 2000
RESULT_REPEATS = 4

SESSION_INIT = 0x40
INIT_ACK = 0x41
IMAGE_REQUEST = 0x42
DIGIT_REQUEST = 0x43
RESULT = 0x44
RESET_SESSION = 0x45
PING = 0x46
PONG = 0x47
STATUS_OK = 0
STATUS_LOW = 1
STATUS_NO_MODEL = 2
STATUS_BAD_SESSION = 3
STATUS_BAD_SLOT = 4
STATUS_INTERNAL = 7


def xor(data, length=None):
    value = 0
    if length is None:
        length = len(data)
    for index in range(length):
        value ^= data[index]
    return value


def put_u32(packet, offset, value):
    packet[offset] = value & 0xFF
    packet[offset + 1] = (value >> 8) & 0xFF
    packet[offset + 2] = (value >> 16) & 0xFF
    packet[offset + 3] = (value >> 24) & 0xFF


def frame(frame_type, session, request, object_type, slot,
          arg0=0, arg1=0, status=STATUS_OK):
    global tx_sequence
    tx_sequence = (tx_sequence + 1) & 0xFF
    packet = bytearray(20)
    packet[0] = 0xA5
    packet[1] = 0x5A
    packet[2] = 0xF2
    packet[3] = 1
    packet[4] = frame_type
    packet[5] = session
    packet[6] = request
    packet[7] = object_type
    packet[8] = slot
    packet[9] = arg0
    packet[10] = arg1
    packet[11] = status
    packet[12] = tx_sequence
    put_u32(packet, 13, FP_BUILD_ID)
    packet[19] = xor(packet, 19)
    return packet


def probe():
    global probe_sequence
    probe_sequence = (probe_sequence + 1) & 0xFF
    packet = bytearray(10)
    packet[0] = 0xA5
    packet[1] = 0x5A
    packet[2] = 0x31
    packet[3] = UART_ID
    packet[4] = probe_sequence
    put_u32(packet, 5, FP_BUILD_ID)
    packet[9] = xor(packet, 9)
    uart.write(packet)


def valid(packet):
    return (len(packet) == 20 and packet[0] == 0xA5 and packet[1] == 0x5A
            and packet[2] == 0xF2 and packet[3] == 1
            and xor(packet, 19) == packet[19])


def setup_uart():
    import cmm
    pins = {
        "hw.-": ("rt117x", "seekfree_art_plus", None, None),
        "uart.12.TXD": ("-", "LPSR_06", Pin("LPSR_06"), None),
        "uart.12.RXD": ("-", "LPSR_07", Pin("LPSR_07"), None),
    }
    cmm.add(pins)
    del pins
    gc.collect()
    return UART(UART_ID, UART_BAUD)


def best(scores):
    best_id = 0
    best_score = scores[0]
    for class_id in range(1, len(scores)):
        if scores[class_id] > best_score:
            best_id = class_id
            best_score = scores[class_id]
    return best_id, best_score


def run_model(path):
    net = None
    try:
        gc.collect()
        net = tf.load(path, load_to_fb=True)
        sums = [0.0] * CLASS_COUNT
        samples = 0
        for unused in range(SAMPLE_COUNT):
            img = sensor.snapshot()
            for item in tf.classify(net, img, min_scale=1.0, scale_mul=0.5,
                                    x_overlap=0.0, y_overlap=0.0):
                scores = item.output()
                if scores:
                    for class_id in range(len(scores)):
                        sums[class_id] += scores[class_id]
                    samples += 1
                    break
            del img
        if samples:
            class_id, score_sum = best(sums)
            return class_id, score_sum / samples, True
    except Exception:
        pass
    finally:
        if net is not None:
            try:
                tf.free_from_fb()
            except Exception:
                pass
            del net
        gc.collect()
    return 0xFF, 0.0, False


def compact_id(raw_id):
    for index in range(len(id_map)):
        if id_map[index] == raw_id:
            return index
    capacity = box_count if box_count >= goal_count else goal_count
    if len(id_map) >= capacity or len(id_map) >= 3:
        return 0xFF
    id_map.append(raw_id)
    return len(id_map) - 1


def send_result(request_packet, raw_id, confidence, status):
    global last_key
    global last_result
    result_id = 0xFF
    if status == STATUS_OK:
        result_id = compact_id(raw_id)
        if result_id == 0xFF:
            status = STATUS_INTERNAL
    percent = int(confidence * 100.0 + 0.5)
    if percent < 0:
        percent = 0
    if percent > 100:
        percent = 100
    packet = frame(RESULT, request_packet[5], request_packet[6],
                   request_packet[7], request_packet[8], result_id, percent, status)
    last_key = (request_packet[5], request_packet[6],
                request_packet[7], request_packet[8])
    last_result = packet
    for unused in range(RESULT_REPEATS):
        uart.write(packet)
        time.sleep_ms(60)


def handle(packet):
    global active_session
    global box_count
    global goal_count
    global id_map
    global last_key
    global last_result
    global pending_request

    frame_type = packet[4]
    if frame_type == SESSION_INIT:
        status = STATUS_OK
        if packet[9] < 1 or packet[9] > 3 or packet[10] != packet[9]:
            status = STATUS_BAD_SLOT
        else:
            active_session = packet[5]
            box_count = packet[9]
            goal_count = packet[10]
            id_map = []
            last_key = None
            last_result = None
        uart.write(frame(INIT_ACK, packet[5], packet[6], 0, 0xFF,
                         packet[9], packet[10], status))
        return
    if frame_type == RESET_SESSION:
        active_session = 0
        box_count = 0
        goal_count = 0
        id_map = []
        last_key = None
        last_result = None
        return
    if frame_type == PING:
        uart.write(frame(PONG, packet[5], packet[6], 0, 0xFF))
        return
    if frame_type != IMAGE_REQUEST and frame_type != DIGIT_REQUEST:
        return

    key = (packet[5], packet[6], packet[7], packet[8])
    if last_key == key and last_result is not None:
        uart.write(last_result)
        return
    if pending_request is not None:
        return
    object_type = 1 if frame_type == IMAGE_REQUEST else 2
    count = box_count if object_type == 1 else goal_count
    if packet[5] == 0 or packet[5] != active_session:
        send_result(packet, 0xFF, 0.0, STATUS_BAD_SESSION)
        return
    if packet[8] >= count:
        send_result(packet, 0xFF, 0.0, STATUS_BAD_SLOT)
        return

    # Defer inference to the module-level loop. Keeping TensorFlow out of the
    # UART parser's nested call stack matches the proven compact diagnostic.
    pending_request = packet
    probe()


def poll_uart():
    global rx_index
    available = uart.any()
    if not available:
        return
    data = uart.read(available)
    if not data:
        return
    for value in data:
        if rx_index == 0:
            if value == 0xA5:
                rx_packet[0] = value
                rx_index = 1
            continue
        if rx_index == 1:
            if value == 0x5A:
                rx_packet[1] = value
                rx_index = 2
            elif value != 0xA5:
                rx_index = 0
            continue
        rx_packet[rx_index] = value
        rx_index += 1
        if rx_index == 20:
            if valid(rx_packet):
                try:
                    handle(bytes(rx_packet))
                except Exception as error:
                    # A malformed command or a port-specific runtime quirk must
                    # not terminate the persistent camera service.
                    print("FP2_HANDLE_ERROR type=%d err=%s" %
                          (rx_packet[4], error))
                    probe()
            rx_index = 0


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(ROI)
sensor.set_framerate(60)
sensor.set_auto_exposure(False, exposure_us=80)
sensor.skip_frames(time=2000)
uart = setup_uart()

active_session = 0
box_count = 0
goal_count = 0
id_map = []
last_key = None
last_result = None
pending_request = None
tx_sequence = 0
probe_sequence = 0
last_probe = None
rx_packet = bytearray(20)
rx_index = 0
gc.collect()

print("#FP2_DIRECT id=%d image=MODEL1 samples=%d threshold=%d deferred=1 mem=%d" %
      (FP_BUILD_ID, SAMPLE_COUNT, int(THRESHOLD * 100), gc.mem_free()))

while True:
    poll_uart()
    if pending_request is not None:
        request = pending_request
        pending_request = None
        model_path = IMAGE_MODEL if request[4] == IMAGE_REQUEST else DIGIT_MODEL
        raw_id, confidence, loaded = run_model(model_path)
        if not loaded:
            result_status = STATUS_NO_MODEL
        elif confidence >= THRESHOLD:
            result_status = STATUS_OK
        else:
            result_status = STATUS_LOW
        send_result(request, raw_id, confidence, result_status)
        del request
        gc.collect()
    now = time.ticks_ms()
    if last_probe is None or time.ticks_diff(now, last_probe) >= PROBE_MS:
        last_probe = now
        probe()
    time.sleep_ms(5)
