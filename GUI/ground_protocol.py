"""Ground-side extension. Legacy FE/F3 is UGV; A4 packets address the future UAV."""
from dataclasses import dataclass, field
import struct
from typing import Optional

GCS, UGV, UAV = 0xFF, 0xFE, 0xFD
NODE_NAMES = {UGV: "UGV", UAV: "UAV"}
ENVELOPE, POLL, WAYPOINT, ACK, UAV_GPS, UAV_DETECTION = 0xA4, 1, 2, 3, 0x10, 0x11

@dataclass
class NodeState:
    node_id: int
    lat: Optional[float] = None
    lon: Optional[float] = None
    heading_rad: Optional[float] = None
    yaw_rate_rad_s: Optional[float] = None
    quality: Optional[int] = None
    fix_valid: Optional[bool] = None
    detected: int = 0
    person_count: int = 0
    received_at: Optional[float] = None
    sequence: Optional[int] = None
    waypoint_id: Optional[int] = None
    waypoint_status: str = "NONE"
    waypoints: list = field(default_factory=list)

    def update(self, message, now):
        if message["node_id"] != self.node_id:
            raise ValueError("node mismatch")
        self.fix_valid = message.get("valid", True)
        if self.fix_valid:
            self.lat, self.lon = message["lat"], message["lon"]
        self.sequence, self.received_at = message["seq"], now
        self.heading_rad = message.get("heading_rad")
        self.yaw_rate_rad_s = message.get("yaw_rate_rad_s")
        self.quality = message.get("quality")
        if self.node_id == UAV:
            self.detected = message.get("detected", 0)
            self.person_count = message.get("person_count", 0)

def waypoint_record(node_id, seq, points):
    if node_id not in NODE_NAMES or not 1 <= seq <= 65535 or not 1 <= len(points) <= 15:
        raise ValueError("invalid waypoint destination, sequence or count")
    payload = bytearray([len(points)])
    for lat, lon in points:
        if not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise ValueError("waypoint out of range")
        payload += struct.pack("<ii", round(lat*1e7), round(lon*1e7))
    raw = struct.pack("<BBBBHB", ENVELOPE, GCS, node_id, WAYPOINT, seq, len(payload)) + payload
    return bytes([GCS, 0xF6, len(raw)]) + raw

def decode_packet(raw):
    if len(raw) in (14, 23) and raw[0] == UGV and raw[2:4] == b"\xf3\x01":
        result = {"type": "gps", "node_id": UGV, "seq": raw[1], "len": 1}
        lat, lon = struct.unpack_from("<ii", raw, 4)
        if len(raw) == 23:
            heading, rate = struct.unpack_from("<ii", raw, 14)
            result.update(heading_rad=heading/1e6, yaw_rate_rad_s=rate/1e6, quality=raw[22])
        # Bytes 12/13 of legacy UGV telemetry are preserved on wire but are not
        # UAV detections. Do not plot a person from a UGV packet.
    elif (7 <= len(raw) <= 128 and raw[0] == ENVELOPE and raw[1] == UAV
          and raw[2] == GCS and len(raw) == 7 + raw[6]):
        kind, seq, length = raw[3], struct.unpack_from("<H", raw, 4)[0], raw[6]
        if kind == ACK and length == 2 and raw[7] == WAYPOINT:
            return {"type": "waypoint_ack", "node_id": UAV, "command_id": seq, "ok": int(raw[8] == 0)}
        if (kind, length) not in ((UAV_GPS, 8), (UAV_GPS, 9), (UAV_DETECTION, 10)):
            return None
        lat, lon = struct.unpack_from("<ii", raw, 7)
        valid = bool(raw[15]) if kind == UAV_GPS and length == 9 else True
        if kind == UAV_GPS and length == 9 and raw[15] not in (0, 1):
            return None
        result = {"type": "gps", "node_id": UAV, "seq": seq, "len": 1,
                  "valid": valid}
        if kind == UAV_DETECTION:
            if raw[15] not in (0, 1):
                return None
            result.update(detected=raw[15], person_count=raw[16])
    else:
        return None
    if not -900000000 <= lat <= 900000000 or not -1800000000 <= lon <= 1800000000:
        return None
    result.update(lat=lat/1e7, lon=lon/1e7)
    return result
