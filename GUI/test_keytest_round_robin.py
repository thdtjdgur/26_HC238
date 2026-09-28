import importlib.util
import sys
import time
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("keytest.py")
SPEC = importlib.util.spec_from_file_location("keytest_round_robin", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
keytest = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = keytest
SPEC.loader.exec_module(keytest)
protocol = sys.modules.get("ground_protocol")
if protocol is None:
    import ground_protocol as protocol


def make_rtcm(message_type: int, fill: int) -> bytes:
    payload = bytes((message_type >> 4, (message_type & 0x0F) << 4, fill))
    body = bytes((0xD3, 0, len(payload))) + payload
    return body + keytest._crc24q(body).to_bytes(3, "big")


class FakeSerial:
    def __init__(self) -> None:
        self.is_open = True
        self.written = bytearray()

    def write(self, data) -> int:
        chunk = bytes(data)
        self.written.extend(chunk)
        return len(chunk)

    @property
    def in_waiting(self) -> int:
        return 0


class RtcmSetCycleTests(unittest.TestCase):
    def setUp(self) -> None:
        keytest._latest_rtcm_frames.clear()
        keytest._rtcm_generation = 0
        keytest._rtcm_replaced_frames = 0
        keytest._pending_waypoints.clear()
        keytest._awaiting_waypoint_ack.clear()
        keytest._rx_messages.clear()
        keytest._cycle_count = 0
        keytest._awaiting_downlink_end = None
        keytest._link_events.clear()
        keytest._ser = None

    def test_all_allowed_types_are_selected_once_in_order(self) -> None:
        now = time.monotonic()
        for index, message_type in enumerate(keytest.RTCM_PASS_ORDER):
            keytest.publish_latest_rtcm_frame(
                message_type, make_rtcm(message_type, index), now
            )

        selected, stale, missing = keytest._take_cycle_inputs()
        self.assertEqual(
            tuple(frame.message_type for frame in selected),
            keytest.RTCM_PASS_ORDER,
        )
        self.assertEqual(stale, ())
        self.assertEqual(missing, ())

        selected, _, missing = keytest._take_cycle_inputs()
        self.assertEqual(selected, [])
        self.assertEqual(missing, keytest.RTCM_PASS_ORDER)

    def test_same_type_overwrites_only_its_own_slot(self) -> None:
        now = time.monotonic()
        old_1004 = make_rtcm(1004, 1)
        new_1004 = make_rtcm(1004, 2)
        frame_1006 = make_rtcm(1006, 3)
        keytest.publish_latest_rtcm_frame(1004, old_1004, now)
        keytest.publish_latest_rtcm_frame(1006, frame_1006, now)
        keytest.publish_latest_rtcm_frame(1004, new_1004, now)

        selected, _, _ = keytest._take_cycle_inputs()
        self.assertEqual([frame.payload for frame in selected], [new_1004, frame_1006])
        self.assertEqual(keytest._rtcm_replaced_frames, 1)

    def test_disallowed_type_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            keytest.publish_latest_rtcm_frame(1020, make_rtcm(1020, 4))

    def test_stale_slot_is_discarded_and_next_fresh_type_is_selected(self) -> None:
        now = time.monotonic()
        keytest.publish_latest_rtcm_frame(
            1004,
            make_rtcm(1004, 5),
            now - keytest.RTCM_MAX_AGE_SECONDS - 1.0,
        )
        keytest.publish_latest_rtcm_frame(1006, make_rtcm(1006, 6), now)

        selected, stale, missing = keytest._take_cycle_inputs()
        self.assertEqual([frame.message_type for frame in selected], [1006])
        self.assertEqual(stale, (1004,))
        self.assertEqual(missing, (1012, 1230))
        self.assertNotIn(1004, keytest._latest_rtcm_frames)

    def test_each_cycle_writes_all_fresh_rtcm_frames(self) -> None:
        now = time.monotonic()
        frame_1004 = make_rtcm(1004, 7)
        frame_1006 = make_rtcm(1006, 8)
        keytest.publish_latest_rtcm_frame(1004, frame_1004, now)
        keytest.publish_latest_rtcm_frame(1006, frame_1006, now)
        fake = FakeSerial()
        keytest._ser = fake

        keytest._run_downlink_cycle()
        self.assertEqual(
            bytes(fake.written),
            keytest._build_rtcm_packet(frame_1004)
            + keytest._build_rtcm_packet(frame_1006)
            + keytest._build_enter_rx_packet(),
        )

    def test_waypoints_are_queued_independently_by_node(self) -> None:
        ugv_id = keytest.queue_waypoints([(37.1, 127.1)], protocol.UGV)
        uav_id = keytest.queue_waypoints([(37.2, 127.2)], protocol.UAV)
        fake = FakeSerial()
        keytest._ser = fake

        keytest._run_downlink_cycle()

        expected = (
            keytest._build_enter_rx_packet()
            + protocol.waypoint_record(protocol.UGV, ugv_id, [(37.1, 127.1)])
            + protocol.waypoint_record(protocol.UAV, uav_id, [(37.2, 127.2)])
        )
        self.assertEqual(bytes(fake.written), expected)

    def test_legacy_ugv_and_addressed_uav_are_distinct(self) -> None:
        ugv = bytearray(23)
        ugv[0:4] = bytes((protocol.UGV, 7, 0xF3, 1))
        import struct
        struct.pack_into("<ii", ugv, 4, 373206600, 1271263100)
        struct.pack_into("<ii", ugv, 14, 123000, -5000)
        ugv[22] = 4

        uav = bytearray(17)
        protocol_header = (protocol.ENVELOPE, protocol.UAV, protocol.GCS,
                           protocol.UAV_DETECTION, 9, 0, 10)
        uav[:7] = bytes(protocol_header)
        struct.pack_into("<ii", uav, 7, 373206700, 1271263200)
        uav[15:17] = bytes((1, 2))

        ugv_msg = protocol.decode_packet(bytes(ugv))
        uav_msg = protocol.decode_packet(bytes(uav))
        self.assertEqual(ugv_msg["node_id"], protocol.UGV)
        self.assertAlmostEqual(ugv_msg["heading_rad"], 0.123)
        self.assertEqual(uav_msg["node_id"], protocol.UAV)
        self.assertEqual(uav_msg["person_count"], 2)


if __name__ == "__main__":
    unittest.main()
