#!/usr/bin/env python3
"""SLAM 可视化采集进程：订阅机器狗 LiDAR 建图话题，写缓存供前端展示。

订阅 /utlidar/grid_map（2D 栅格地图，PointCloud2）与 /utlidar/robot_pose（位姿），
每 500ms 写 slam_cache.json。建图开关由部署侧（LiDAR switch 命令）控制，
本进程只做"订阅 → 缓存 → 前端可视化"，不负责开启建图。
"""
import json, os, struct, sys, time
import rclpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

CACHE_FILE = "/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor/slam_cache.json"

latest_map = {}
latest_pose = {}


def parse_grid(msg, max_points=6000):
    """解析栅格地图点云：每点 (x, y, z, intensity=占用)。"""
    offsets = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z", "intensity"):
            offsets[f.name] = f.offset
    if not all(k in offsets for k in ("x", "y", "z")):
        return None
    data = msg.data
    ps = msg.point_step or 32
    n = msg.width
    step = max(1, n // max_points)
    pts = []
    for i in range(0, n, step):
        base = i * ps
        try:
            x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + offsets["z"])[0]
            it = struct.unpack_from("<f", data, base + offsets["intensity"])[0] if "intensity" in offsets else 0.0
            pts.append([round(x, 3), round(y, 3), round(z, 3), round(it, 4)])
        except struct.error:
            break
    return pts


def main():
    print("[slam_worker] starting", flush=True)
    rclpy.init()
    node = rclpy.create_node("go2w_slam_worker")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)

    def cb_map(msg):
        latest_map["msg"] = msg

    def cb_pose(msg):
        latest_pose["msg"] = msg

    node.create_subscription(PointCloud2, "/utlidar/grid_map", cb_map, qos)
    node.create_subscription(PoseStamped, "/utlidar/robot_pose", cb_pose, qos)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    print("[slam_worker] subscribed grid_map + robot_pose, waiting for data", flush=True)
    last_write = 0
    while True:
        executor.spin_once(timeout_sec=0.2)
        now = time.time()
        if now - last_write > 0.5:
            payload = {"online": False, "map_ts": 0, "pose_ts": 0, "grid": [], "pose": None}
            if "msg" in latest_map:
                grid = parse_grid(latest_map["msg"])
                if grid:
                    payload["online"] = True
                    payload["grid"] = grid
                    payload["map_ts"] = now
            if "msg" in latest_pose:
                p = latest_pose["msg"].pose
                payload["pose"] = {"x": round(p.position.x, 3), "y": round(p.position.y, 3),
                                   "z": round(p.position.z, 3)}
                payload["pose_ts"] = now
            tmp = CACHE_FILE + ".tmp"
            with open(tmp, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, CACHE_FILE)
            last_write = now


if __name__ == "__main__":
    main()
