import gc
import machine
import os
import time
import fp2_common as fp


FP_BUILD_ID = 7200136
FAKE_RECOGNITION_ENABLE = False
FAKE_RECOGNITION_DELAY_MS = 1000
PROBE_PERIOD_MS = 2000
JOB_COMMIT_RETRY_COUNT = 3
JOB_COMMIT_RETRY_DELAY_MS = 100
RESET_HANDOFF_DELAY_MS = 300
TRACE_FILE = "/sd/fp2_trace.bin"
RESULT_REPLAY_COUNT = 4
RESULT_REPLAY_INITIAL_DELAY_MS = 80
RESULT_REPLAY_INTERVAL_MS = 120

SESSION_INIT = 0x40
INIT_ACK = 0x41
IMAGE_REQUEST = 0x42
DIGIT_REQUEST = 0x43
RESET_SESSION = 0x45
PING = 0x46
PONG = 0x47
OBJECT_NONE = 0
OBJECT_BOX = 1
OBJECT_GOAL = 2


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


def send_frame(frame_type, session_id, request_id, object_type, object_slot,
               arg0=0, arg1=0, status=fp.STATUS_OK):
    global tx_sequence
    tx_sequence = (tx_sequence + 1) & 0xFF
    packet = fp.make_frame(
        FP_BUILD_ID, tx_sequence, frame_type, session_id, request_id,
        object_type, object_slot, arg0, arg1, status,
    )
    uart.write(packet)
    return packet


def save_and_send_result(session_id, request_id, object_type, object_slot,
                         result_id, confidence, status):
    percent = int(confidence * 100.0 + 0.5)
    if percent < 0:
        percent = 0
    if percent > 100:
        percent = 100
    packet = send_frame(
        fp.TYPE_RESULT, session_id, request_id, object_type, object_slot,
        result_id, percent, status,
    )
    fp.write_file(fp.LAST_FILE, packet)
    return packet


def reset_runtime_session(remove_state):
    global active_session
    global box_count
    global goal_count
    global fake_box_ids
    global fake_goal_ids
    global pending_fake
    global last_result
    global replay_remaining

    active_session = 0
    box_count = 0
    goal_count = 0
    fake_box_ids = []
    fake_goal_ids = []
    pending_fake = None
    last_result = None
    replay_remaining = 0
    fp.remove_file(fp.JOB_FILE)
    fp.remove_file(fp.LAST_FILE)
    if remove_state:
        fp.remove_file(fp.STATE_FILE)


def handle_session(packet):
    global active_session
    global box_count
    global goal_count
    global fake_box_ids
    global fake_goal_ids
    global pending_fake
    global last_result
    global replay_remaining

    session_id = packet[5]
    new_box_count = packet[9]
    new_goal_count = packet[10]
    status = fp.STATUS_OK
    if new_box_count < 1 or new_box_count > 3 or new_goal_count != new_box_count:
        status = fp.STATUS_BAD_SLOT
    else:
        active_session = session_id
        box_count = new_box_count
        goal_count = new_goal_count
        pending_fake = None
        last_result = None
        replay_remaining = 0
        fp.remove_file(fp.JOB_FILE)
        fp.remove_file(fp.LAST_FILE)
        fp.write_state(fp.new_state(session_id, box_count, goal_count))
        if FAKE_RECOGNITION_ENABLE:
            fake_box_ids = shuffled_ids(box_count)
            fake_goal_ids = shuffled_ids(goal_count)
    send_frame(
        INIT_ACK, session_id, packet[6], OBJECT_NONE, 0xFF,
        new_box_count, new_goal_count, status,
    )
    print("FP_SESSION id=%d s=%d n=%d status=%d fake=%d" %
          (FP_BUILD_ID, session_id, new_box_count, status,
           1 if FAKE_RECOGNITION_ENABLE else 0))


def duplicate_result(packet):
    global last_result

    if last_result is None:
        last_result = fp.read_file(fp.LAST_FILE)
    if not fp.valid_frame(last_result) or last_result[4] != fp.TYPE_RESULT:
        return False
    if (last_result[5] == packet[5] and last_result[6] == packet[6]
            and last_result[7] == packet[7] and last_result[8] == packet[8]):
        uart.write(last_result)
        return True
    return False


def commit_job(packet):
    expected = bytes(packet)
    for attempt in range(JOB_COMMIT_RETRY_COUNT):
        try:
            fp.write_file(fp.JOB_FILE, expected)
            try:
                os.sync()
            except Exception:
                pass
            if fp.read_file(fp.JOB_FILE) == expected:
                return True
        except Exception as error:
            print("FP_JOB write failed attempt=%d err=%s" % (attempt + 1, error))
        time.sleep_ms(JOB_COMMIT_RETRY_DELAY_MS)
    return False


def restart_for_worker(frame_type):
    # 0xA1/0xA2 means the listener committed an image/digit job and requested
    # an interpreter restart. The worker overwrites this as soon as it starts.
    marker = 0xA1 if frame_type == IMAGE_REQUEST else 0xA2
    try:
        fp.write_file(TRACE_FILE, bytes((marker,)))
        try:
            os.sync()
        except Exception:
            pass
    except Exception:
        pass
    time.sleep_ms(RESET_HANDOFF_DELAY_MS)

    # This OpenART build did not reliably hand control back to main.py after
    # machine.reset(). A soft reset explicitly clears the Python heap and
    # reruns the boot/main sequence, which is exactly what worker isolation needs.
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


def handle_request(packet):
    global pending_fake

    if duplicate_result(packet):
        return
    session_id = packet[5]
    request_id = packet[6]
    object_type = OBJECT_BOX if packet[4] == IMAGE_REQUEST else OBJECT_GOAL
    object_slot = packet[8]
    if session_id == 0 or session_id != active_session:
        save_and_send_result(session_id, request_id, object_type, object_slot,
                             0xFF, 0.0, fp.STATUS_BAD_SESSION)
        return
    count = box_count if object_type == OBJECT_BOX else goal_count
    if object_slot >= count:
        save_and_send_result(session_id, request_id, object_type, object_slot,
                             0xFF, 0.0, fp.STATUS_BAD_SLOT)
        return

    if FAKE_RECOGNITION_ENABLE:
        if pending_fake is not None:
            save_and_send_result(session_id, request_id, object_type, object_slot,
                                 0xFF, 0.0, fp.STATUS_BUSY)
            return
        pending_fake = (
            session_id, request_id, object_type, object_slot,
            time.ticks_add(time.ticks_ms(), FAKE_RECOGNITION_DELAY_MS),
        )
        return

    if not commit_job(packet):
        save_and_send_result(
            session_id, request_id, object_type, object_slot,
            0xFF, 0.0, fp.STATUS_INTERNAL_ERROR,
        )
        print("FP_JOB id=%d commit=FAILED s=%d req=%d type=%d slot=%d" %
              (FP_BUILD_ID, session_id, request_id, object_type, object_slot))
        return
    print("FP_JOB id=%d s=%d req=%d type=%d slot=%d" %
          (FP_BUILD_ID, session_id, request_id, object_type, object_slot))
    restart_for_worker(packet[4])


def handle_packet(packet):
    if not fp.valid_frame(packet):
        return
    frame_type = packet[4]
    if frame_type == SESSION_INIT:
        handle_session(packet)
    elif frame_type == IMAGE_REQUEST or frame_type == DIGIT_REQUEST:
        handle_request(packet)
    elif frame_type == RESET_SESSION:
        reset_runtime_session(True)
    elif frame_type == PING:
        send_frame(PONG, packet[5], packet[6], OBJECT_NONE, 0xFF)


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
            if value == fp.HEADER_0:
                rx_packet[0] = value
                rx_index = 1
            continue
        if rx_index == 1:
            if value == fp.HEADER_1:
                rx_packet[1] = value
                rx_index = 2
            elif value != fp.HEADER_0:
                rx_index = 0
            continue
        rx_packet[rx_index] = value
        rx_index += 1
        if rx_index == fp.PACKET_SIZE:
            handle_packet(rx_packet)
            rx_index = 0


def process_fake():
    global pending_fake
    global last_result

    if pending_fake is None or time.ticks_diff(time.ticks_ms(), pending_fake[4]) < 0:
        return
    session_id, request_id, object_type, object_slot, deadline = pending_fake
    pending_fake = None
    mapping = fake_box_ids if object_type == OBJECT_BOX else fake_goal_ids
    last_result = save_and_send_result(
        session_id, request_id, object_type, object_slot,
        mapping[object_slot], 1.0, fp.STATUS_OK,
    )


def process_result_replay():
    global replay_remaining
    global replay_next_tick

    if replay_remaining <= 0:
        return
    now = time.ticks_ms()
    if time.ticks_diff(now, replay_next_tick) < 0:
        return
    if (not fp.valid_frame(last_result)
            or last_result[4] != fp.TYPE_RESULT
            or last_result[5] != active_session):
        replay_remaining = 0
        return
    uart.write(last_result)
    replay_remaining -= 1
    replay_next_tick = time.ticks_add(now, RESULT_REPLAY_INTERVAL_MS)


def send_probe():
    global probe_sequence
    global last_probe_tick

    now = time.ticks_ms()
    if last_probe_tick is not None and time.ticks_diff(now, last_probe_tick) < PROBE_PERIOD_MS:
        return
    last_probe_tick = now
    probe_sequence = (probe_sequence + 1) & 0xFF
    uart.write(fp.make_probe(FP_BUILD_ID, probe_sequence))


uart = fp.configure_uart()
state = fp.read_state()
active_session = state[0]
box_count = state[1]
goal_count = state[2]
fake_box_ids = []
fake_goal_ids = []
pending_fake = None
last_result = fp.read_file(fp.LAST_FILE)
replay_remaining = (RESULT_REPLAY_COUNT
                    if fp.valid_frame(last_result)
                    and last_result[4] == fp.TYPE_RESULT
                    and last_result[5] == active_session
                    else 0)
replay_next_tick = time.ticks_add(
    time.ticks_ms(), RESULT_REPLAY_INITIAL_DELAY_MS,
)
rx_packet = bytearray(fp.PACKET_SIZE)
rx_index = 0
tx_sequence = 0
probe_sequence = 0
last_probe_tick = None
random_state = (time.ticks_ms() ^ FP_BUILD_ID) & 0xFFFFFFFF
gc.collect()

print("#FP2_LISTENER id=%d fake=%d session=%d mem=%d" %
      (FP_BUILD_ID, 1 if FAKE_RECOGNITION_ENABLE else 0,
       active_session, gc.mem_free()))

while True:
    poll_uart()
    process_fake()
    process_result_replay()
    send_probe()
    time.sleep_ms(5)
