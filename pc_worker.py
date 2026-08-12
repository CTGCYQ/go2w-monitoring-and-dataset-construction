#!/usr/bin/env python3
"""常驻点云采集进程：订阅 /utlidar/cloud，写最新点云到缓存文件。

独立进程运行（避免 rclpy 与 uvicorn asyncio 冲突），server.py 读缓存文件。
"""
import json, os, struct, sys, time
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

CACHE_FILE = "/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor/pc_cache.json"
latest = {}

def parse_cloud(msg, max_points=4000):
    offsets = {}
    has_intensity = False
    for f in msg.fields:
        if f.name in ("x","y","z","intensity"):
            offsets[f.name] = f.offset
            if f.name == "intensity":
                has_intensity = True
    if not all(k in offsets for k in ("x","y","z")):
        return None
    data = msg.data
    ps = msg.point_step or 32
    n = msg.width
    step = max(1, n // max_points)
    pts = []
    for i in range(0, n, step):
        base = i * ps
        try:
            x = struct.unpack_from("<f", data, base+offsets["x"])[0]
            y = struct.unpack_from("<f", data, base+offsets["y"])[0]
            z = struct.unpack_from("<f", data, base+offsets["z"])[0]
            if has_intensity:
                it = struct.unpack_from("<f", data, base+offsets["intensity"])[0]
                pts.append([round(x,3), round(y,3), round(z,3), round(it,4)])
            else:
                pts.append([round(x,3), round(y,3), round(z,3)])
        except struct.error:
            break
    return {"points": pts, "count": len(pts), "total": n, "has_intensity": has_intensity}

def main():
    print("[pc_worker] starting", flush=True)
    rclpy.init()
    node = rclpy.create_node("go2w_pc_worker")
    def cb(msg):
        latest["msg"] = msg
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_ALL, durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(PointCloud2, "/utlidar/cloud", cb, qos)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    print("[pc_worker] subscribed, waiting for data", flush=True)
    last_write = 0
    while True:
        executor.spin_once(timeout_sec=0.2)
        if "msg" in latest:
            now = time.time()
            if now - last_write > 0.5:  # 每 500ms 写一次缓存
                parsed = parse_cloud(latest["msg"])
                if parsed:
                    payload = {"online": True, "ts": now, **parsed}
                    tmp = CACHE_FILE + ".tmp"
                    with open(tmp, "w") as f:
                        json.dump(payload, f)
                    os.replace(tmp, CACHE_FILE)
                    last_write = now

if __name__ == "__main__":
    main()
