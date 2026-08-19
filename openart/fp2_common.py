import gc
import os
from machine import Pin
from machine import UART


UART_ID = 12
UART_BAUD = 115200
JOB_FILE = "/sd/fp2_job.bin"
STATE_FILE = "/sd/fp2_state.bin"
LAST_FILE = "/sd/fp2_last.bin"

HEADER_0 = 0xA5
HEADER_1 = 0x5A
DOMAIN = 0xF2
VERSION = 0x01
PACKET_SIZE = 20
TYPE_PROBE = 0x31
TYPE_RESULT = 0x44

STATUS_OK = 0
STATUS_LOW_CONFIDENCE = 1
STATUS_MODEL_UNAVAILABLE = 2
STATUS_BAD_SESSION = 3
STATUS_BAD_SLOT = 4
STATUS_BUSY = 5
STATUS_BAD_FRAME = 6
STATUS_INTERNAL_ERROR = 7

_CMM_LINES = (
    "hw,-,-,rt117x,seekfree_art_plus,",
    "uart,12,TXD,-,LPSR_06,",
    "uart,12,RXD,-,LPSR_07,",
)


def xor_bytes(data, length=None):
    if length is None:
        length = len(data)
    value = 0
    for index in range(length):
        value ^= data[index]
    return value


def append_u32(packet, value):
    packet.append(value & 0xFF)
    packet.append((value >> 8) & 0xFF)
    packet.append((value >> 16) & 0xFF)
    packet.append((value >> 24) & 0xFF)


def make_probe(build_id, sequence):
    packet = bytearray((HEADER_0, HEADER_1, TYPE_PROBE, UART_ID, sequence & 0xFF))
    append_u32(packet, build_id)
    packet.append(xor_bytes(packet))
    return packet


def make_frame(build_id, sequence, frame_type, session_id, request_id,
               object_type, object_slot, arg0, arg1, status):
    packet = bytearray((
        HEADER_0, HEADER_1, DOMAIN, VERSION, frame_type & 0xFF,
        session_id & 0xFF, request_id & 0xFF, object_type & 0xFF,
        object_slot & 0xFF, arg0 & 0xFF, arg1 & 0xFF,
        status & 0xFF, sequence & 0xFF,
    ))
    append_u32(packet, build_id)
    packet.append(0)
    packet.append(0)
    packet.append(xor_bytes(packet))
    return packet


def valid_frame(packet):
    return (
        packet is not None
        and len(packet) == PACKET_SIZE
        and packet[0] == HEADER_0
        and packet[1] == HEADER_1
        and packet[2] == DOMAIN
        and packet[3] == VERSION
        and xor_bytes(packet, PACKET_SIZE - 1) == packet[PACKET_SIZE - 1]
    )


def configure_uart():
    import cmm

    pin_map = {}
    for line in _CMM_LINES:
        parts = line.split(",")
        name = parts[0] + "." + (parts[2] if parts[1] == "-" else parts[1] + "." + parts[2])
        try:
            pin = Pin(parts[4])
        except Exception:
            pin = None
        pin_map[name] = (parts[3], parts[4], pin, None)
    cmm.add(pin_map)
    del pin_map
    gc.collect()
    return UART(UART_ID, UART_BAUD)


def read_file(path):
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except Exception:
        return None


def write_file(path, data):
    with open(path, "wb") as handle:
        handle.write(data)


def remove_file(path):
    try:
        os.remove(path)
    except Exception:
        pass


def new_state(session_id=0, box_count=0, goal_count=0):
    return [session_id, box_count, goal_count, []]


def read_state():
    data = read_file(STATE_FILE)
    if data is None or len(data) != 13 or data[0] != 0xF2 or data[1] != 1:
        return new_state()
    if xor_bytes(data, 12) != data[12] or data[5] > 3:
        return new_state()
    mapping = []
    for index in range(data[5]):
        offset = 6 + index * 2
        mapping.append((data[offset], data[offset + 1]))
    return [data[2], data[3], data[4], mapping]


def write_state(state):
    packet = bytearray((0xF2, 1, state[0] & 0xFF, state[1] & 0xFF,
                        state[2] & 0xFF, len(state[3]) & 0xFF))
    for index in range(3):
        if index < len(state[3]):
            packet.append(state[3][index][0] & 0xFF)
            packet.append(state[3][index][1] & 0xFF)
        else:
            packet.append(0xFF)
            packet.append(0xFF)
    packet.append(xor_bytes(packet))
    write_file(STATE_FILE, packet)


def compact_id(state, raw_id):
    for pair in state[3]:
        if pair[0] == raw_id:
            return pair[1], True
    capacity = state[1]
    if state[2] > capacity:
        capacity = state[2]
    if len(state[3]) >= capacity or len(state[3]) >= 3:
        return 0xFF, False
    result_id = len(state[3])
    state[3].append((raw_id, result_id))
    write_state(state)
    return result_id, True
