from __future__ import annotations

import sys
import types
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CAMERA_SOURCE = ROOT / "openart" / "openart_roi_classifier.py"


class FakeUART:
    def __init__(self, *_args):
        self.rx = bytearray()
        self.tx: list[bytes] = []

    def any(self):
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data):
        self.tx.append(bytes(data))
        return len(data)


class FakeTime:
    def __init__(self):
        self.now = 100

    def ticks_ms(self):
        return self.now

    @staticmethod
    def ticks_add(value, delta):
        return value + delta

    @staticmethod
    def ticks_diff(left, right):
        return left - right


def load_camera_protocol():
    sensor_module = types.ModuleType("sensor")
    machine_module = types.ModuleType("machine")
    machine_module.UART = FakeUART
    sys.modules["sensor"] = sensor_module
    sys.modules["machine"] = machine_module

    source = CAMERA_SOURCE.read_text(encoding="utf-8")
    protocol_source = source.split("\nsensor.reset()", 1)[0]
    namespace = {"__name__": "fp2_camera_protocol_test"}
    exec(compile(protocol_source, str(CAMERA_SOURCE), "exec"), namespace)
    namespace["time"] = FakeTime()
    namespace["fp_uart"] = FakeUART()
    namespace["rx_packet"] = bytearray(namespace["FP2_PACKET_SIZE"])
    namespace["rx_index"] = 0
    namespace["tx_sequence"] = 0
    namespace["random_state"] = 0x12345678
    namespace["image_net"] = None
    namespace["image_labels"] = []
    namespace["image_model_attempted"] = False
    namespace["active_session"] = 0
    namespace["session_box_count"] = 0
    namespace["session_goal_count"] = 0
    namespace["box_id_by_slot"] = []
    namespace["goal_id_by_slot"] = []
    namespace["pending_request"] = None
    namespace["last_result_key"] = None
    namespace["last_result_packet"] = None
    return namespace


def validate_packet(ns, packet, frame_type):
    assert len(packet) == ns["FP2_PACKET_SIZE"]
    assert packet[:4] == bytes((0xA5, 0x5A, 0xF2, 0x01))
    assert packet[4] == frame_type
    assert ns["xor_bytes"](packet, len(packet) - 1) == packet[-1]


def run_regression():
    ns = load_camera_protocol()
    make_packet = ns["make_fp2_packet"]
    uart: FakeUART = ns["fp_uart"]

    init = make_packet(ns["FP2_SESSION_INIT"], 7, 0,
                       ns["FP_OBJECT_NONE"], 0xFF, 3, 3,
                       ns["FP_STATUS_OK"], 1, ns["MCU_BUILD_ID"])
    uart.rx.extend(init[:7])
    ns["poll_uart_rx"]()
    assert not uart.tx
    uart.rx.extend(init[7:])
    ns["poll_uart_rx"]()
    validate_packet(ns, uart.tx[-1], ns["FP2_INIT_ACK"])
    assert ns["active_session"] == 7
    assert sorted(ns["box_id_by_slot"]) == [0, 1, 2]
    assert sorted(ns["goal_id_by_slot"]) == [0, 1, 2]

    request = make_packet(ns["FP2_IMAGE_REQUEST"], 7, 9,
                          ns["FP_OBJECT_BOX"], 1, 0, 0,
                          ns["FP_STATUS_OK"], 2, ns["MCU_BUILD_ID"])
    ns["handle_fp2_packet"](request)
    before = len(uart.tx)
    ns["process_pending_request"](object())
    assert len(uart.tx) == before
    ns["time"].now += ns["FAKE_RECOGNITION_DELAY_MS"]
    ns["process_pending_request"](object())
    result = uart.tx[-1]
    validate_packet(ns, result, ns["FP2_RESULT"])
    assert result[5:9] == bytes((7, 9, ns["FP_OBJECT_BOX"], 1))
    assert result[9] == ns["box_id_by_slot"][1]
    assert result[10] == 100 and result[11] == ns["FP_STATUS_OK"]

    ns["handle_fp2_packet"](request)
    assert uart.tx[-1] == result

    goal_request = make_packet(ns["FP2_DIGIT_REQUEST"], 7, 10,
                               ns["FP_OBJECT_GOAL"], 2, 0, 0,
                               ns["FP_STATUS_OK"], 3,
                               ns["MCU_BUILD_ID"])
    ns["handle_fp2_packet"](goal_request)
    ns["time"].now += ns["FAKE_RECOGNITION_DELAY_MS"]
    ns["process_pending_request"](object())
    goal_result = uart.tx[-1]
    validate_packet(ns, goal_result, ns["FP2_RESULT"])
    assert goal_result[9] == ns["goal_id_by_slot"][2]

    stale = make_packet(ns["FP2_IMAGE_REQUEST"], 8, 11,
                        ns["FP_OBJECT_BOX"], 0, 0, 0,
                        ns["FP_STATUS_OK"], 4, ns["MCU_BUILD_ID"])
    ns["handle_fp2_packet"](stale)
    validate_packet(ns, uart.tx[-1], ns["FP2_RESULT"])
    assert uart.tx[-1][11] == ns["FP_STATUS_BAD_SESSION"]

    print(
        "FP2_REGRESSION_OK packet=20 fragmented_rx=1 fake_delay_ms=%d "
        "box_ids=%s goal_ids=%s duplicate_idempotent=1 stale_rejected=1"
        % (
            ns["FAKE_RECOGNITION_DELAY_MS"],
            ns["box_id_by_slot"],
            ns["goal_id_by_slot"],
        )
    )


if __name__ == "__main__":
    run_regression()
