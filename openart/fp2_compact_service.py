import gc
import sensor
import time
import tf
from machine import UART


# User configuration.
FAKE_RECOGNITION_ENABLE = True
FAKE_RECOGNITION_DELAY_MS = 1000
IMAGE_CONFIDENCE_THRESHOLD = 0.80
DIGIT_CONFIDENCE_THRESHOLD = 0.80
AUTO_IMAGE_TEST_ENABLE = True
AUTO_IMAGE_TEST_PERIOD_MS = 2000

# Camera crop copied from roi_capture(2).py (QVGA coordinates).
ROI_X = 55
ROI_Y = 40
ROI_W = 245
ROI_H = 180
ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

IMAGE_MODEL_1 = "/sd/smartcar_class_model_qat_1.tflite"
IMAGE_MODEL_2 = "/sd/smartcar_class_model_qat_2.tflite"
IMAGE_LABEL_FILE = "/sd/model_labels.txt"
DIGIT_MODEL = "/sd/smartcar_digit_model_qat.tflite"
DIGIT_LABEL_FILE = "/sd/digit_labels.txt"

FP_BUILD_ID = 7200121
MCU_BUILD_ID = 7100959
FP_UART_ID = 12
UART_BAUD = 115200
FP_PROBE_PERIOD_MS = 2000

FP_HEADER_0 = 0xA5
FP_HEADER_1 = 0x5A
FP_TYPE_PORT_PROBE = 0x31
FP2_DOMAIN = 0xF2
FP2_VERSION = 0x01
FP2_PACKET_SIZE = 20

FP2_SESSION_INIT = 0x40
FP2_INIT_ACK = 0x41
FP2_IMAGE_REQUEST = 0x42
FP2_DIGIT_REQUEST = 0x43
FP2_RESULT = 0x44
FP2_RESET_SESSION = 0x45
FP2_PING = 0x46
FP2_PONG = 0x47

FP_OBJECT_NONE = 0
FP_OBJECT_BOX = 1
FP_OBJECT_GOAL = 2

FP_STATUS_OK = 0
FP_STATUS_LOW_CONFIDENCE = 1
FP_STATUS_MODEL_UNAVAILABLE = 2
FP_STATUS_BAD_SESSION = 3
FP_STATUS_BAD_SLOT = 4
FP_STATUS_BUSY = 5
FP_STATUS_BAD_FRAME = 6
FP_STATUS_INTERNAL_ERROR = 7

_CMM_LINES = (
    "hw,-,-,rt117x,seekfree_art_plus,",
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


def read_u32_le(data, offset):
    return (
        data[offset]
        | (data[offset + 1] << 8)
        | (data[offset + 2] << 16)
        | (data[offset + 3] << 24)
    )


def xor_bytes(data, length=None):
    value = 0
    if length is None:
        length = len(data)
    for index in range(length):
        value ^= data[index]
    return value


def make_probe_packet(sequence):
    packet = bytearray((
        FP_HEADER_0,
        FP_HEADER_1,
        FP_TYPE_PORT_PROBE,
        FP_UART_ID,
        sequence,
    ))
    append_u32_le(packet, FP_BUILD_ID)
    packet.append(xor_bytes(packet))
    return packet


def make_fp2_packet(frame_type, session_id, request_id, object_type,
                    object_slot, arg0, arg1, status, sequence, sender_id):
    packet = bytearray((
        FP_HEADER_0,
        FP_HEADER_1,
        FP2_DOMAIN,
        FP2_VERSION,
        frame_type & 0xFF,
        session_id & 0xFF,
        request_id & 0xFF,
        object_type & 0xFF,
        object_slot & 0xFF,
        arg0 & 0xFF,
        arg1 & 0xFF,
        status & 0xFF,
        sequence & 0xFF,
    ))
    append_u32_le(packet, sender_id)
    packet.append(0)
    packet.append(0)
    packet.append(xor_bytes(packet))
    return packet


def file_exists(path):
    handle = None
    try:
        handle = open(path, "rb")
        handle.close()
        return True
    except Exception:
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass
        return False


def load_labels(path):
    labels = []
    if path is None:
        return labels
    with open(path, "r") as label_file:
        for line in label_file:
            label = line.strip()
            if label:
                labels.append(label)
    return labels


def highest_score(scores):
    best_index = 0
    best_score = scores[0]
    for index in range(1, len(scores)):
        if scores[index] > best_score:
            best_index = index
            best_score = scores[index]
    return best_index, best_score


def next_random():
    global random_state
    random_state ^= (random_state << 13) & 0xFFFFFFFF
    random_state ^= random_state >> 17
    random_state ^= (random_state << 5) & 0xFFFFFFFF
    random_state &= 0xFFFFFFFF
    return random_state


def shuffled_ids(count):
    values = [index for index in range(count)]
    index = count - 1
    while index > 0:
        swap = next_random() % (index + 1)
        values[index], values[swap] = values[swap], values[index]
        index -= 1
    return values


def send_frame(frame_type, session_id, request_id, object_type,
               object_slot, arg0=0, arg1=0, status=FP_STATUS_OK):
    global tx_sequence
    tx_sequence = (tx_sequence + 1) & 0xFF
    packet = make_fp2_packet(
        frame_type,
        session_id,
        request_id,
        object_type,
        object_slot,
        arg0,
        arg1,
        status,
        tx_sequence,
        FP_BUILD_ID,
    )
    fp_uart.write(packet)
    return packet


def send_result(session_id, request_id, object_type, object_slot,
                result_id, confidence, status):
    global last_result_key
    global last_result_packet

    confidence_percent = int(confidence * 100.0 + 0.5)
    if confidence_percent < 0:
        confidence_percent = 0
    if confidence_percent > 100:
        confidence_percent = 100
    last_result_packet = send_frame(
        FP2_RESULT,
        session_id,
        request_id,
        object_type,
        object_slot,
        result_id,
        confidence_percent,
        status,
    )
    last_result_key = (session_id, request_id, object_type, object_slot)
    print(
        "FP_RESULT fp_id=%d session=%d req=%d type=%d slot=%d id=%d conf=%d status=%d fake=%d"
        % (
            FP_BUILD_ID,
            session_id,
            request_id,
            object_type,
            object_slot,
            result_id,
            confidence_percent,
            status,
            1 if FAKE_RECOGNITION_ENABLE else 0,
        )
    )


def ensure_labels():
    global image_labels
    global digit_labels

    try:
        if not image_labels and file_exists(IMAGE_LABEL_FILE):
            image_labels = load_labels(IMAGE_LABEL_FILE)
        if not digit_labels and file_exists(DIGIT_LABEL_FILE):
            digit_labels = load_labels(DIGIT_LABEL_FILE)
    except Exception as error:
        print("FP labels failed: %s" % error)
    gc.collect()


def classify_model(model_path):
    if not file_exists(model_path):
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE

    gc.collect()
    net = None
    img = None
    try:
        net = tf.load(model_path, load_to_fb=True)
        img = sensor.snapshot()
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
        if best_id == 0xFF:
            return 0xFF, 0.0, FP_STATUS_INTERNAL_ERROR
        return best_id, best_confidence, FP_STATUS_OK
    except Exception as error:
        print("FP inference failed: %s" % error)
        return 0xFF, 0.0, FP_STATUS_INTERNAL_ERROR
    finally:
        if img is not None:
            del img
        gc.collect()
        if net is not None:
            try:
                tf.free_from_fb()
            except Exception as error:
                print("FP free fb failed: %s" % error)
            del net
        gc.collect()


def classify_image_ensemble():
    ensure_labels()
    if not image_labels:
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE

    result_1 = classify_model(IMAGE_MODEL_1)
    result_2 = classify_model(IMAGE_MODEL_2)
    valid_1 = result_1[2] == FP_STATUS_OK and result_1[1] >= IMAGE_CONFIDENCE_THRESHOLD
    valid_2 = result_2[2] == FP_STATUS_OK and result_2[1] >= IMAGE_CONFIDENCE_THRESHOLD

    if valid_1 and valid_2:
        return result_1 if result_1[1] >= result_2[1] else result_2
    if valid_1:
        return result_1
    if valid_2:
        return result_2
    if result_1[0] != 0xFF or result_2[0] != 0xFF:
        chosen = result_1 if result_1[1] >= result_2[1] else result_2
        return chosen[0], chosen[1], FP_STATUS_LOW_CONFIDENCE
    return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE


def classify_digit_once():
    ensure_labels()
    if not digit_labels:
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE
    class_id, confidence, status = classify_model(DIGIT_MODEL)
    if class_id != 0xFF and class_id < len(digit_labels):
        try:
            class_id = int(digit_labels[class_id])
        except Exception:
            pass
    if status == FP_STATUS_OK and confidence < DIGIT_CONFIDENCE_THRESHOLD:
        status = FP_STATUS_LOW_CONFIDENCE
    return class_id, confidence, status


def compact_real_result(raw_id, confidence, status):
    global real_id_map

    if status != FP_STATUS_OK or raw_id == 0xFF:
        return raw_id, confidence, status
    if raw_id in real_id_map:
        return real_id_map[raw_id], confidence, status
    capacity = session_box_count
    if session_goal_count > capacity:
        capacity = session_goal_count
    if len(real_id_map) >= capacity:
        return 0xFF, confidence, FP_STATUS_INTERNAL_ERROR
    compact_id = len(real_id_map)
    real_id_map[raw_id] = compact_id
    return compact_id, confidence, status


def reset_session():
    global active_session
    global session_box_count
    global session_goal_count
    global box_id_by_slot
    global goal_id_by_slot
    global pending_request
    global last_result_key
    global last_result_packet
    global real_id_map

    active_session = 0
    session_box_count = 0
    session_goal_count = 0
    box_id_by_slot = []
    goal_id_by_slot = []
    pending_request = None
    last_result_key = None
    last_result_packet = None
    real_id_map = {}


def handle_session_init(packet):
    global active_session
    global session_box_count
    global session_goal_count
    global box_id_by_slot
    global goal_id_by_slot
    global pending_request
    global last_result_key
    global last_result_packet
    global real_id_map

    session_id = packet[5]
    box_count = packet[9]
    goal_count = packet[10]
    status = FP_STATUS_OK
    if box_count < 1 or box_count > 3 or goal_count != box_count:
        status = FP_STATUS_BAD_SLOT
    elif (session_id != active_session or box_count != session_box_count or
          goal_count != session_goal_count):
        active_session = session_id
        session_box_count = box_count
        session_goal_count = goal_count
        pending_request = None
        last_result_key = None
        last_result_packet = None
        real_id_map = {}
        if FAKE_RECOGNITION_ENABLE:
            box_id_by_slot = shuffled_ids(box_count)
            goal_id_by_slot = shuffled_ids(goal_count)
        else:
            box_id_by_slot = []
            goal_id_by_slot = []
    send_frame(
        FP2_INIT_ACK,
        session_id,
        packet[6],
        FP_OBJECT_NONE,
        0xFF,
        box_count,
        goal_count,
        status,
    )
    print(
        "FP_SESSION fp_id=%d session=%d box=%d goal=%d status=%d fake=%d box_ids=%s goal_ids=%s"
        % (
            FP_BUILD_ID,
            session_id,
            box_count,
            goal_count,
            status,
            1 if FAKE_RECOGNITION_ENABLE else 0,
            str(box_id_by_slot),
            str(goal_id_by_slot),
        )
    )


def handle_recognition_request(packet):
    global pending_request

    session_id = packet[5]
    request_id = packet[6]
    object_type = FP_OBJECT_BOX if packet[4] == FP2_IMAGE_REQUEST else FP_OBJECT_GOAL
    object_slot = packet[8]
    request_key = (session_id, request_id, object_type, object_slot)

    if last_result_key == request_key and last_result_packet is not None:
        fp_uart.write(last_result_packet)
        print("FP duplicate result resent session=%d req=%d" % (session_id, request_id))
        return
    if session_id == 0 or session_id != active_session:
        send_result(session_id, request_id, object_type, object_slot,
                    0xFF, 0.0, FP_STATUS_BAD_SESSION)
        return
    count = session_box_count if object_type == FP_OBJECT_BOX else session_goal_count
    if object_slot >= count:
        send_result(session_id, request_id, object_type, object_slot,
                    0xFF, 0.0, FP_STATUS_BAD_SLOT)
        return
    if pending_request is not None:
        send_result(session_id, request_id, object_type, object_slot,
                    0xFF, 0.0, FP_STATUS_BUSY)
        return

    delay_ms = FAKE_RECOGNITION_DELAY_MS if FAKE_RECOGNITION_ENABLE else 0
    pending_request = {
        "session": session_id,
        "request": request_id,
        "type": object_type,
        "slot": object_slot,
        "deadline": time.ticks_add(time.ticks_ms(), delay_ms),
    }
    print(
        "FP_REQUEST session=%d req=%d type=%d slot=%d delay=%dms"
        % (session_id, request_id, object_type, object_slot, delay_ms)
    )


def handle_fp2_packet(packet):
    if packet[2] != FP2_DOMAIN or packet[3] != FP2_VERSION:
        return
    if xor_bytes(packet, FP2_PACKET_SIZE - 1) != packet[FP2_PACKET_SIZE - 1]:
        print("FP RX checksum error")
        return
    frame_type = packet[4]
    if frame_type == FP2_SESSION_INIT:
        handle_session_init(packet)
    elif frame_type == FP2_IMAGE_REQUEST or frame_type == FP2_DIGIT_REQUEST:
        handle_recognition_request(packet)
    elif frame_type == FP2_RESET_SESSION:
        reset_session()
        print("FP session reset")
    elif frame_type == FP2_PING:
        send_frame(FP2_PONG, packet[5], packet[6], FP_OBJECT_NONE,
                   0xFF, 0, 0, FP_STATUS_OK)


def poll_uart_rx():
    global rx_index

    available = fp_uart.any()
    if not available:
        return
    data = fp_uart.read(available)
    if not data:
        return
    for value in data:
        if rx_index == 0:
            if value == FP_HEADER_0:
                rx_packet[0] = value
                rx_index = 1
            continue
        if rx_index == 1:
            if value == FP_HEADER_1:
                rx_packet[1] = value
                rx_index = 2
            elif value == FP_HEADER_0:
                rx_packet[0] = value
            else:
                rx_index = 0
            continue
        rx_packet[rx_index] = value
        rx_index += 1
        if rx_index >= FP2_PACKET_SIZE:
            handle_fp2_packet(rx_packet)
            rx_index = 0


def process_pending_request():
    global pending_request

    if pending_request is None:
        return
    if time.ticks_diff(time.ticks_ms(), pending_request["deadline"]) < 0:
        return
    session_id = pending_request["session"]
    request_id = pending_request["request"]
    object_type = pending_request["type"]
    object_slot = pending_request["slot"]
    pending_request = None

    if FAKE_RECOGNITION_ENABLE:
        mapping = box_id_by_slot if object_type == FP_OBJECT_BOX else goal_id_by_slot
        send_result(session_id, request_id, object_type, object_slot,
                    mapping[object_slot], 1.0, FP_STATUS_OK)
    elif object_type == FP_OBJECT_BOX:
        raw_id, confidence, status = classify_image_ensemble()
        result_id, confidence, status = compact_real_result(
            raw_id, confidence, status
        )
        send_result(session_id, request_id, object_type, object_slot,
                    result_id, confidence, status)
    else:
        raw_id, confidence, status = classify_digit_once()
        result_id, confidence, status = compact_real_result(
            raw_id, confidence, status
        )
        send_result(session_id, request_id, object_type, object_slot,
                    result_id, confidence, status)


def process_auto_image_test():
    global last_auto_test_tick

    if not AUTO_IMAGE_TEST_ENABLE or pending_request is not None:
        return
    now = time.ticks_ms()
    if (last_auto_test_tick is not None and
            time.ticks_diff(now, last_auto_test_tick) < AUTO_IMAGE_TEST_PERIOD_MS):
        return
    last_auto_test_tick = now
    result_id, confidence, status = classify_image_ensemble()
    print(
        "FP_AUTO_IMAGE fp_id=%d id=%d conf=%.4f status=%d mem=%d"
        % (FP_BUILD_ID, result_id, confidence, status, gc.mem_free())
    )


def send_periodic_probe():
    global probe_sequence
    global last_probe_tick

    now = time.ticks_ms()
    if last_probe_tick is not None and time.ticks_diff(now, last_probe_tick) < FP_PROBE_PERIOD_MS:
        return
    last_probe_tick = now
    probe_sequence = (probe_sequence + 1) & 0xFF
    fp_uart.write(make_probe_packet(probe_sequence))


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing(ROI)
sensor.set_framerate(60)
sensor.set_auto_exposure(False, exposure_us=80)
sensor.skip_frames(time=2000)
sensor.set_vflip(True)

try:
    load_cmm_config()
    print("FP cmm.add OK")
except Exception as error:
    print("FP cmm.add failed: %s" % error)

fp_uart = UART(FP_UART_ID, UART_BAUD)
rx_packet = bytearray(FP2_PACKET_SIZE)
rx_index = 0
probe_sequence = 0
tx_sequence = 0
last_probe_tick = None
random_state = (time.ticks_ms() ^ FP_BUILD_ID) & 0xFFFFFFFF
image_labels = []
digit_labels = []
active_session = 0
session_box_count = 0
session_goal_count = 0
box_id_by_slot = []
goal_id_by_slot = []
pending_request = None
last_result_key = None
last_result_packet = None
real_id_map = {}
last_auto_test_tick = None

print("========== ROI Classifier Ready ==========")
print(
    "FP_BUILD id=%d uart=%d protocol=FP2_COMPACT fake=%d delay=%dms auto_test=%d/%dms"
    % (
        FP_BUILD_ID,
        FP_UART_ID,
        1 if FAKE_RECOGNITION_ENABLE else 0,
        FAKE_RECOGNITION_DELAY_MS,
        1 if AUTO_IMAGE_TEST_ENABLE else 0,
        AUTO_IMAGE_TEST_PERIOD_MS,
    )
)
print("ROI: x=%d y=%d w=%d h=%d" % ROI)
print("==========================================")

while True:
    poll_uart_rx()
    send_periodic_probe()
    process_pending_request()
    process_auto_image_test()
    time.sleep_ms(5)
