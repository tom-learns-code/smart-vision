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
MODEL_LOAD_TO_FB = True
IMAGE_MODELS_SEQUENTIAL = True

# OpenMV IDE serial-terminal debug. Keep False for competition.
IDE_DEBUG_ENABLE = True
IDE_DEBUG_PERIOD_MS = 500
IDE_DEBUG_MODE_IMAGE = 1
IDE_DEBUG_MODE_DIGIT = 2
IDE_DEBUG_MODE_BOTH = 3
IDE_DEBUG_MODE = IDE_DEBUG_MODE_IMAGE
IDE_DEBUG_LEGACY_MODEL_ONLY = False

# Camera crop copied from roi_capture(2).py (QVGA coordinates).
ROI_X = 55
ROI_Y = 40
ROI_W = 245
ROI_H = 180
ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

IMAGE_MODEL_1_CANDIDATES = (
    "/sd/smartcar_class_model_qat_1.tflite",
    "/flash/smartcar_class_model_qat_1.tflite",
    "/sdcard/smartcar_class_model_qat_1.tflite",
    "/smartcar_class_model_qat_1.tflite",
    "smartcar_class_model_qat_1.tflite",
)
IMAGE_MODEL_2_CANDIDATES = (
    "/sd/smartcar_class_model_qat_2.tflite",
    "/flash/smartcar_class_model_qat_2.tflite",
    "/sdcard/smartcar_class_model_qat_2.tflite",
    "/smartcar_class_model_qat_2.tflite",
    "smartcar_class_model_qat_2.tflite",
)
LEGACY_IMAGE_MODEL_CANDIDATES = (
    "/sd/mobilenet.tflite",
    "/flash/mobilenet.tflite",
    "/sdcard/mobilenet.tflite",
    "/mobilenet.tflite",
    "mobilenet.tflite",
)
IMAGE_LABEL_CANDIDATES = (
    "/sd/model_labels.txt",
    "/flash/model_labels.txt",
    "/sdcard/model_labels.txt",
    "/model_labels.txt",
    "model_labels.txt",
)

DIGIT_MODEL_CANDIDATES = (
    "/sd/smartcar_digit_model_qat.tflite",
    "/flash/smartcar_digit_model_qat.tflite",
    "/sdcard/smartcar_digit_model_qat.tflite",
    "/smartcar_digit_model_qat.tflite",
    "smartcar_digit_model_qat.tflite",
)
DIGIT_LABEL_CANDIDATES = (
    "/sd/digit_labels.txt",
    "/flash/digit_labels.txt",
    "/sdcard/digit_labels.txt",
    "/digit_labels.txt",
    "digit_labels.txt",
)

FP_BUILD_ID = 7200119
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


def find_existing_file(candidates, file_kind="file"):
    for path in candidates:
        handle = None
        try:
            handle = open(path, "rb")
            handle.close()
            if IDE_DEBUG_ENABLE:
                print("FP_FILE kind=%s found=%s" % (file_kind, path))
            return path
        except Exception as error:
            if handle is not None:
                try:
                    handle.close()
                except Exception:
                    pass
            if IDE_DEBUG_ENABLE:
                print("FP_FILE kind=%s miss=%s err=%s" % (file_kind, path, error))
    if IDE_DEBUG_ENABLE:
        print("FP_FILE kind=%s result=NOT_FOUND" % file_kind)
    return None


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


def load_model_file(candidates, model_name):
    model_path = find_existing_file(candidates, model_name + "_model")
    if model_path is None:
        print("FP %s model unavailable" % model_name)
        return None
    try:
        gc.collect()
        if MODEL_LOAD_TO_FB:
            try:
                print("FP loading %s model to fb: %s" % (model_name, model_path))
                return tf.load(model_path, load_to_fb=True)
            except Exception as error:
                print("FP %s fb load failed: %s" % (model_name, error))
                gc.collect()
                return None
        print("FP loading %s model from file: %s" % (model_name, model_path))
        return tf.load(model_path, load_to_fb=False)
    except Exception as error:
        print("FP %s model load failed: %s" % (model_name, error))
        return None


def load_image_models_if_needed():
    global image_labels
    global image_models_attempted

    if image_models_attempted:
        return bool(image_labels)
    image_models_attempted = True
    label_path = find_existing_file(IMAGE_LABEL_CANDIDATES, "image_labels")
    if label_path is None:
        print("FP image labels unavailable")
        image_models_attempted = False
        return False
    try:
        image_labels = load_labels(label_path)
        if not image_labels:
            print("FP image label file is empty")
            image_models_attempted = False
            return False
        print(
            "FP image resources ready sequential=%d labels=%d"
            % (1 if IMAGE_MODELS_SEQUENTIAL else 0, len(image_labels))
        )
        return True
    except Exception as error:
        print("FP image resources setup failed: %s" % error)
        image_labels = []
        image_models_attempted = False
        return False


def classify_image_model_file(candidates, model_name):
    stage_started = time.ticks_ms()
    print("FP_IMAGE_STAGE model=%s phase=LOAD_BEGIN" % model_name)
    net = load_model_file(candidates, model_name)
    load_ms = time.ticks_diff(time.ticks_ms(), stage_started)
    if net is None:
        print(
            "FP_IMAGE_STAGE model=%s phase=LOAD_FAILED load=%dms"
            % (model_name, load_ms)
        )
        return (0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE), load_ms

    img = None
    capture_ms = 0
    inference_ms = 0
    try:
        print("FP_IMAGE_STAGE model=%s phase=CAPTURE_BEGIN load=%dms" % (model_name, load_ms))
        capture_started = time.ticks_ms()
        img = sensor.snapshot()
        capture_ms = time.ticks_diff(time.ticks_ms(), capture_started)
        print(
            "FP_IMAGE_STAGE model=%s phase=INFER_BEGIN load=%dms capture=%dms"
            % (model_name, load_ms, capture_ms)
        )
        inference_started = time.ticks_ms()
        result = classify_loaded_model(net, img)
        inference_ms = time.ticks_diff(time.ticks_ms(), inference_started)
    finally:
        if img is not None:
            del img
        gc.collect()
        try:
            tf.free_from_fb()
        except Exception as error:
            print("FP_IMAGE_STAGE model=%s phase=FREE_FB_FAIL err=%s" % (model_name, error))
        del net
        gc.collect()
    total_ms = time.ticks_diff(time.ticks_ms(), stage_started)
    print(
        "FP_IMAGE_STAGE model=%s phase=RELEASED id=%d conf=%.4f status=%d load=%dms capture=%dms infer=%dms total=%dms"
        % (
            model_name, result[0], result[1], result[2],
            load_ms, capture_ms, inference_ms, total_ms,
        )
    )
    return result, total_ms


def load_digit_resources_if_needed():
    global digit_labels
    global digit_model_attempted

    if digit_model_attempted:
        return bool(digit_labels)
    digit_model_attempted = True
    label_path = find_existing_file(DIGIT_LABEL_CANDIDATES, "digit_labels")
    if label_path is None:
        print("FP digit labels unavailable")
        digit_model_attempted = False
        return False
    try:
        digit_labels = load_labels(label_path)
        if not digit_labels:
            print("FP digit label file is empty")
            digit_model_attempted = False
            return False
        print("FP digit resources ready labels=%d" % len(digit_labels))
        return True
    except Exception as error:
        print("FP digit resources setup failed: %s" % error)
        digit_labels = []
        digit_model_attempted = False
        return False


def classify_loaded_model(net, img):
    if net is None:
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE
    try:
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
                class_index, confidence = highest_score(scores)
                if confidence > best_confidence:
                    best_id = class_index
                    best_confidence = confidence
        if best_id == 0xFF:
            return 0xFF, 0.0, FP_STATUS_INTERNAL_ERROR
        return best_id, best_confidence, FP_STATUS_OK
    except Exception as error:
        print("FP model inference failed: %s" % error)
        return 0xFF, 0.0, FP_STATUS_INTERNAL_ERROR


def label_name(labels, result_id):
    if result_id >= 0 and result_id < len(labels):
        return labels[result_id]
    return "?"


def result_valid(result, threshold):
    return (
        result[2] == FP_STATUS_OK
        and result[0] != 0xFF
        and result[1] >= threshold
    )


def fuse_image_results(result_1, result_2):
    valid_1 = result_valid(result_1, IMAGE_CONFIDENCE_THRESHOLD)
    valid_2 = result_valid(result_2, IMAGE_CONFIDENCE_THRESHOLD)

    if valid_1 and valid_2:
        if result_1[0] == result_2[0]:
            chosen = result_1 if result_1[1] >= result_2[1] else result_2
            return chosen[0], chosen[1], FP_STATUS_OK, "AGREE"
        chosen = result_1 if result_1[1] >= result_2[1] else result_2
        return chosen[0], chosen[1], FP_STATUS_OK, "DISAGREE_HIGHER"
    if valid_1:
        return result_1[0], result_1[1], FP_STATUS_OK, "MODEL1_ONLY"
    if valid_2:
        return result_2[0], result_2[1], FP_STATUS_OK, "MODEL2_ONLY"

    candidate_1 = result_1[0] != 0xFF
    candidate_2 = result_2[0] != 0xFF
    if candidate_1 or candidate_2:
        if candidate_1 and (not candidate_2 or result_1[1] >= result_2[1]):
            return result_1[0], result_1[1], FP_STATUS_LOW_CONFIDENCE, "BOTH_LOW"
        return result_2[0], result_2[1], FP_STATUS_LOW_CONFIDENCE, "BOTH_LOW"
    if (
        result_1[2] == FP_STATUS_MODEL_UNAVAILABLE
        and result_2[2] == FP_STATUS_MODEL_UNAVAILABLE
    ):
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE, "NO_MODEL"
    return 0xFF, 0.0, FP_STATUS_INTERNAL_ERROR, "NO_RESULT"


def classify_image_ensemble():
    started = time.ticks_ms()
    if not load_image_models_if_needed():
        result_1 = (0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE)
        result_2 = (0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE)
        model_1_ms = 0
        model_2_ms = 0
    else:
        result_1, model_1_ms = classify_image_model_file(
            IMAGE_MODEL_1_CANDIDATES, "image1"
        )
        result_2, model_2_ms = classify_image_model_file(
            IMAGE_MODEL_2_CANDIDATES, "image2"
        )

    result_id, confidence, status, reason = fuse_image_results(result_1, result_2)
    total_ms = time.ticks_diff(time.ticks_ms(), started)
    print(
        "FP_IMAGE_FUSE m1=%d/%s/%.4f/s%d/%dms m2=%d/%s/%.4f/s%d/%dms final=%d/%s/%.4f/s%d reason=%s total=%dms"
        % (
            result_1[0], label_name(image_labels, result_1[0]),
            result_1[1], result_1[2], model_1_ms,
            result_2[0], label_name(image_labels, result_2[0]),
            result_2[1], result_2[2], model_2_ms,
            result_id, label_name(image_labels, result_id),
            confidence, status, reason, total_ms,
        )
    )
    return result_id, confidence, status


def classify_ide_image():
    if not IDE_DEBUG_LEGACY_MODEL_ONLY:
        return classify_image_ensemble()
    if not load_image_models_if_needed():
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE
    result, total_ms = classify_image_model_file(
        LEGACY_IMAGE_MODEL_CANDIDATES, "legacy"
    )
    print(
        "FP_IMAGE_LEGACY id=%d label=%s conf=%.4f status=%d total=%dms"
        % (
            result[0], label_name(image_labels, result[0]), result[1],
            result[2], total_ms,
        )
    )
    return result


def digit_result_id(class_index):
    if class_index < 0 or class_index >= len(digit_labels):
        return class_index
    try:
        value = int(digit_labels[class_index])
        if value >= 0 and value <= 255:
            return value
    except Exception:
        pass
    return class_index


def classify_digit_once():
    started = time.ticks_ms()
    if not load_digit_resources_if_needed():
        return 0xFF, 0.0, FP_STATUS_MODEL_UNAVAILABLE
    raw_result, model_ms = classify_image_model_file(
        DIGIT_MODEL_CANDIDATES, "digit"
    )
    class_index, confidence, status = raw_result
    if class_index != 0xFF:
        result_id = digit_result_id(class_index)
    else:
        result_id = 0xFF
    if status == FP_STATUS_OK and confidence < DIGIT_CONFIDENCE_THRESHOLD:
        status = FP_STATUS_LOW_CONFIDENCE
    elapsed_ms = time.ticks_diff(time.ticks_ms(), started)
    print(
        "FP_DIGIT model_class=%d label=%s id=%d conf=%.4f status=%d model=%dms total=%dms"
        % (
            class_index, label_name(digit_labels, class_index), result_id,
            confidence, status, model_ms, elapsed_ms,
        )
    )
    return result_id, confidence, status


def compact_real_result(raw_id, confidence, status):
    global real_id_map

    if status != FP_STATUS_OK or raw_id == 0xFF:
        return raw_id, confidence, status
    if raw_id in real_id_map:
        compact_id = real_id_map[raw_id]
        print("FP_ID_MAP raw=%d compact=%d existing=1" % (raw_id, compact_id))
        return compact_id, confidence, status

    capacity = session_box_count
    if session_goal_count > capacity:
        capacity = session_goal_count
    if len(real_id_map) >= capacity:
        print(
            "FP_ID_MAP overflow raw=%d used=%d capacity=%d map=%s"
            % (raw_id, len(real_id_map), capacity, str(real_id_map))
        )
        return 0xFF, confidence, FP_STATUS_INTERNAL_ERROR

    compact_id = len(real_id_map)
    real_id_map[raw_id] = compact_id
    print(
        "FP_ID_MAP raw=%d compact=%d existing=0 map=%s"
        % (raw_id, compact_id, str(real_id_map))
    )
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
        "FP_SESSION fp_id=%d session=%d box=%d goal=%d status=%d fake=%d box_ids=%s goal_ids=%s real_map=%s"
        % (
            FP_BUILD_ID,
            session_id,
            box_count,
            goal_count,
            status,
            1 if FAKE_RECOGNITION_ENABLE else 0,
            str(box_id_by_slot),
            str(goal_id_by_slot),
            str(real_id_map),
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
        return False
    if time.ticks_diff(time.ticks_ms(), pending_request["deadline"]) < 0:
        return False
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
    return True


def process_ide_debug(request_processed):
    global last_ide_debug_tick

    if not IDE_DEBUG_ENABLE or request_processed:
        return
    now = time.ticks_ms()
    if (
        last_ide_debug_tick is not None
        and time.ticks_diff(now, last_ide_debug_tick) < IDE_DEBUG_PERIOD_MS
    ):
        return
    last_ide_debug_tick = now

    if IDE_DEBUG_MODE == IDE_DEBUG_MODE_IMAGE or IDE_DEBUG_MODE == IDE_DEBUG_MODE_BOTH:
        result_id, confidence, status = classify_ide_image()
        print(
            "FP_IDE_IMAGE fp_id=%d id=%d label=%s conf=%.4f status=%d"
            % (
                FP_BUILD_ID, result_id, label_name(image_labels, result_id),
                confidence, status,
            )
        )
    if IDE_DEBUG_MODE == IDE_DEBUG_MODE_DIGIT or IDE_DEBUG_MODE == IDE_DEBUG_MODE_BOTH:
        result_id, confidence, status = classify_digit_once()
        print(
            "FP_IDE_DIGIT fp_id=%d id=%d label=%s conf=%.4f status=%d"
            % (
                FP_BUILD_ID, result_id, label_name(digit_labels, result_id),
                confidence, status,
            )
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
image_models_attempted = False
digit_labels = []
digit_model_attempted = False
active_session = 0
session_box_count = 0
session_goal_count = 0
box_id_by_slot = []
goal_id_by_slot = []
pending_request = None
last_result_key = None
last_result_packet = None
real_id_map = {}
last_ide_debug_tick = None

print("========== ROI Classifier Ready ==========")
print(
    "FP_BUILD id=%d uart=%d protocol=FP2 fake=%d delay=%dms image_models=2/sequential%d image_threshold=%d digit_models=1 digit_threshold=%d load_to_fb=%d ide_debug=%d/%d/%dms legacy_ab=%d"
    % (
        FP_BUILD_ID,
        FP_UART_ID,
        1 if FAKE_RECOGNITION_ENABLE else 0,
        FAKE_RECOGNITION_DELAY_MS,
        1 if IMAGE_MODELS_SEQUENTIAL else 0,
        int(IMAGE_CONFIDENCE_THRESHOLD * 100.0 + 0.5),
        int(DIGIT_CONFIDENCE_THRESHOLD * 100.0 + 0.5),
        1 if MODEL_LOAD_TO_FB else 0,
        1 if IDE_DEBUG_ENABLE else 0,
        IDE_DEBUG_MODE,
        IDE_DEBUG_PERIOD_MS,
        1 if IDE_DEBUG_LEGACY_MODEL_ONLY else 0,
    )
)
print("ROI: x=%d y=%d w=%d h=%d" % ROI)
print("==========================================")

while True:
    poll_uart_rx()
    send_periodic_probe()
    request_processed = process_pending_request()
    process_ide_debug(request_processed)
    time.sleep_ms(5)
