"""Latest aligned Z16 frame over a dock SSH channel; RAM only."""
import json
import math
import struct
import threading
import time

MAX_HEADER = 16384
MAX_PAYLOAD = 1280 * 720 * 2


def recv_exact(channel, size):
    parts = bytearray()
    while len(parts) < size:
        chunk = channel.recv(size - len(parts))
        if not chunk:
            raise EOFError("depth transport closed")
        parts.extend(chunk)
    return bytes(parts)


def read_frame(channel):
    length = struct.unpack("!I", recv_exact(channel, 4))[0]
    if not 0 < length <= MAX_HEADER:
        raise ValueError("missing or oversized depth header")
    header_bytes = recv_exact(channel, length)
    h = json.loads(header_bytes)
    if not isinstance(h, dict) or type(h.get("version")) is not int or h.get("version") != 1 or h.get("format") != "Z16_LE" or h.get("alignment") != "color":
        raise ValueError("unsupported depth format")
    for key in ("width", "height", "payload_bytes", "frame_number"):
        if type(h.get(key)) is not int or h[key] <= 0:
            raise ValueError("invalid depth dimensions or sequence")
    size = h["width"] * h["height"] * 2
    if size != h["payload_bytes"] or size > MAX_PAYLOAD:
        raise ValueError("invalid depth payload size")
    for key in ("meters_per_unit", "source_age_ms", "captured_host_time_s", "device_timestamp_ms", "color_timestamp_ms", "fx", "fy", "ppx", "ppy"):
        v = h.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError("invalid depth metadata")
    if not 0 < h["meters_per_unit"] <= .01 or h["fx"] <= 0 or h["fy"] <= 0:
        raise ValueError("invalid metric scale or focal length")
    if not 0 <= h["source_age_ms"] <= 1500:
        raise ValueError("depth frame stale")
    if not 0 <= h["ppx"] < h["width"] or not 0 <= h["ppy"] < h["height"]:
        raise ValueError("invalid principal point")
    if h.get("frame_id") != "realsense_color_optical_frame":
        raise ValueError("invalid optical frame")
    coeffs = h.get("coefficients")
    if (not isinstance(coeffs, list) or len(coeffs) != 5 or
        any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in coeffs)):
        raise ValueError("invalid distortion coefficients")
    pixels = recv_exact(channel, size)
    return h, pixels, struct.pack("!I", length) + header_bytes + pixels


def summarize(header, pixels):
    w, h = header["width"], header["height"]
    values = []
    for y in range(max(0, h//2-2), min(h, h//2+3)):
        for x in range(max(0, w//2-2), min(w, w//2+3)):
            raw = struct.unpack_from("<H", pixels, (y*w+x)*2)[0]
            if raw:
                values.append(raw * header["meters_per_unit"])
    values.sort()
    return {"online": True, **header,
            "center_depth_m": values[len(values)//2] if values else None,
            "center_valid_pixels": len(values), "raw_depth_saved": False}


def packet_with_age(packet, arrival_age_s):
    length = struct.unpack("!I", packet[:4])[0]
    header = json.loads(packet[4:4+length])
    header["source_age_ms"] += max(0, arrival_age_s) * 1000
    encoded = json.dumps(header, allow_nan=False).encode()
    return struct.pack("!I", len(encoded)) + encoded + packet[4+length:]


class DepthReceiver:
    def __init__(self, host, username, password):
        self.host, self.username, self.password = host, username, password
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.frame = None
        self.thread = None

    def start(self):
        with self.lock:
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, daemon=True, name="metric-depth")
                self.thread.start()

    def stop(self):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(timeout=3)

    def snapshot(self):
        with self.lock:
            frame = self.frame
        if frame is None:
            return {"online": False, "reason": "depth unavailable"}, None
        summary, packet, received = frame
        age = time.monotonic() - received
        if age + summary["source_age_ms"] / 1000 > 1.5:
            return {"online": False, "reason": "depth stale"}, None
        return {**summary, "arrival_age_s": round(age, 4)}, packet

    def _run(self):
        import paramiko
        while not self.stopping.is_set():
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                client.connect(self.host, username=self.username, password=self.password,
                               timeout=2, banner_timeout=2, auth_timeout=2)
                channel = client.get_transport().open_channel(
                    "direct-tcpip", ("127.0.0.1", 8766), ("127.0.0.1", 0), timeout=2)
                channel.settimeout(2)
                last = None
                while not self.stopping.is_set():
                    started = time.monotonic()
                    channel.sendall(b"D")
                    header, pixels, packet = read_frame(channel)
                    key = (header["frame_number"], header["captured_host_time_s"])
                    # Re-reading one frame never refreshes its receiver age.
                    if key != last:
                        summary = summarize(header, pixels)
                        with self.lock:
                            self.frame = (summary, packet, started)
                        last = key
                    self.stopping.wait(.2)
            except Exception:
                with self.lock:
                    self.frame = None
                self.stopping.wait(1)
            finally:
                client.close()
