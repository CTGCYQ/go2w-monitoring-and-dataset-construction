#!/usr/bin/env python3
"""常驻世界地图重建进程：订阅点云 + Point-LIO 位姿，累积成全局环境地图。

与 pc_worker.py 同一套路：独立进程（避免 rclpy 与 uvicorn asyncio 冲突），
把每帧 LiDAR 点云经 /odom_corrected 位姿变换到统一世界系叠加，体素降采样后
写 map_cache.json，供前端「走一圈重建」实时展示。

数据链：/utlidar/cloud（雷达点云）+ /odom_corrected（Point-LIO 位姿）
   → 相对位姿累积 → 全局地图（强度 + 高度伪彩） + 机器人轨迹
   → /mnt/.../go2w-monitor/map_cache.json

坐标系说明：odom_corrected 的 header.frame_id = "lidar_odom"，与雷达点云
同源自洽。选取第一帧位姿为世界锚点归零，因此绝对坐标漂移不影响相对重建。
"""
import json, os, struct, math, time

import rclpy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

CACHE_FILE = "/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor/map_cache.json"

# 世界地图体素分辨率（米）。越小越精细、点越多越卡；0.1m 兼顾精细与性能。
VOXEL = 0.10
# 相机/建模盲区：距雷达过近的点（<0.5m，对应 Point-LIO blind）与过远点数过滤。
MIN_RANGE = 0.5
MAX_RANGE = 30.0
# 单帧降采样上限（点太多时均匀抽样）。
FRAME_MAX_POINTS = 6000

latest_cloud = {}
latest_odom = {}


def _q_to_matrix(qx, qy, qz, qw):
    """四元数 → 3x3 旋转矩阵（row-major，用于 R@p）。"""
    n = 1.0 / math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx * n, qy * n, qz * n, qw * n
    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)
    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx * qx + qz * qz)
    r12 = 2 * (qy * qz - qx * qw)
    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx * qx + qy * qy)
    return ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22))


def parse_cloud(msg, R, t):
    """解析点云并变换到世界系。R/t 为当前帧位姿的重力对齐旋转与平移。

    返回 (world_pts, has_intensity)，world_pts 每点为 [x,y,z,intensity|None]。
    """
    offsets = {}
    has_intensity = False
    for f in msg.fields:
        if f.name in ("x", "y", "z", "intensity"):
            offsets[f.name] = f.offset
            if f.name == "intensity":
                has_intensity = True
    if not all(k in offsets for k in ("x", "y", "z")):
        return None, False
    data = msg.data
    ps = msg.point_step or 32
    n = msg.width
    step = max(1, n // FRAME_MAX_POINTS)
    pts = []
    for i in range(0, n, step):
        base = i * ps
        try:
            x = struct.unpack_from("<f", data, base + offsets["x"])[0]
            y = struct.unpack_from("<f", data, base + offsets["y"])[0]
            z = struct.unpack_from("<f", data, base + offsets["z"])[0]
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            r = math.sqrt(x * x + y * y + z * z)
            if r < MIN_RANGE or r > MAX_RANGE:
                continue
            # 世界系：先旋转再平移
            wx = R[0][0] * x + R[0][1] * y + R[0][2] * z + t[0]
            wy = R[1][0] * x + R[1][1] * y + R[1][2] * z + t[1]
            wz = R[2][0] * x + R[2][1] * y + R[2][2] * z + t[2]
            if has_intensity:
                it = struct.unpack_from("<f", data, base + offsets["intensity"])[0]
                if not math.isfinite(it):
                    continue
                pts.append([round(wx, 3), round(wy, 3), round(wz, 3), round(it, 4)])
            else:
                pts.append([round(wx, 3), round(wy, 3), round(wz, 3)])
        except struct.error:
            break
    return pts, has_intensity


class WorldMap:
    """全局地图累积：体素降采样 + 高度/强度统计。"""

    def __init__(self, voxel=VOXEL):
        self.voxel = voxel
        self.cells = {}          # (ix,iy,iz) -> [x,y,z, intensity_sum, intensity_n]
        self.traj = []           # 轨迹点 [x,y,z]
        self.count_cloud = 0
        self.count_points = 0

    def add_points(self, pts, has_intensity):
        self.count_cloud += 1
        self.count_points += len(pts)
        for p in pts:
            x, y, z = p[0], p[1], p[2]
            key = (int(math.floor(x / self.voxel)),
                   int(math.floor(y / self.voxel)),
                   int(math.floor(z / self.voxel)))
            cell = self.cells.get(key)
            if cell is None:
                it = p[3] if has_intensity else 0.0
                self.cells[key] = [x, y, z, it, 1]
            else:
                cell[0] = (cell[0] * cell[4] + x) / (cell[4] + 1)
                cell[1] = (cell[1] * cell[4] + y) / (cell[4] + 1)
                cell[2] = (cell[2] * cell[4] + z) / (cell[4] + 1)
                if has_intensity:
                    cell[3] += p[3]
                cell[4] += 1

    def add_traj(self, p):
        self.traj.append([round(p[0], 3), round(p[1], 3), round(p[2], 3)])

    def to_payload(self):
        pts = []
        for cell in self.cells.values():
            # 平均强度（无强度时用 0，后续按高度着色）
            it = cell[3] / cell[4] if cell[4] else 0.0
            pts.append([round(cell[0], 3), round(cell[1], 3),
                        round(cell[2], 3), round(it, 4)])
        return {"points": pts, "trajectory": self.traj,
                "count": len(pts), "cloud_count": self.count_cloud,
                "point_count": self.count_points, "voxel": self.voxel}


def main():
    print("[map_worker] starting", flush=True)
    rclpy.init()
    node = rclpy.create_node("go2w_map_worker")

    def cb_cloud(msg):
        latest_cloud["msg"] = msg
        latest_cloud["t"] = time.monotonic()

    def cb_odom(msg):
        latest_odom["msg"] = msg
        latest_odom["t"] = time.monotonic()

    # /utlidar/cloud 由机器狗以 BEST_EFFORT 发布；/odom_corrected 由 Point-LIO
    # 以 RELIABLE 发布。两类必须分别对应，否则 QoS 不兼容收不到消息。
    qos_cloud = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(PointCloud2, "/utlidar/cloud", cb_cloud, qos_cloud)
    node.create_subscription(Odometry, "/odom_corrected", cb_odom, qos_odom)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    print("[map_worker] subscribed cloud(BEST_EFFORT) + odom(RELIABLE), waiting", flush=True)

    world = WorldMap()
    anchor = None          # 首帧位姿 (R, t)，世界系归零锚点
    last_write = 0
    last_traj = 0

    while True:
        executor.spin_once(timeout_sec=0.2)
        now = time.time()

        # 位置新鲜度：位姿与点云都要新鲜（沿用 pc_worker 的 1.5s 阈值）
        cloud_fresh = ("msg" in latest_cloud and
                       time.monotonic() - latest_cloud["t"] <= 1.5)
        odom_fresh = ("msg" in latest_odom and
                      time.monotonic() - latest_odom["t"] <= 1.5)

        if cloud_fresh and odom_fresh:
            om = latest_odom["msg"].pose.pose
            q = om.orientation
            R = _q_to_matrix(q.x, q.y, q.z, q.w)
            p = om.position
            t = (p.x, p.y, p.z)

            if anchor is None:
                # 锚点：首帧位姿，把世界系原点定在机器人初始位置
                anchor = (R, t)
                # 初始锚点时点云放在原点附近
                pts, has_intensity = parse_cloud(latest_cloud["msg"], R, t)
                if pts:
                    world.add_points(pts, has_intensity)
                world.add_traj((0.0, 0.0, 0.0))
            else:
                # 相对锚点的位姿：R_rel = R * R_anchor^T, t_rel = t - R_rel @ t_anchor
                Ra, ta = anchor
                # Ra^T
                RaT = ((Ra[0][0], Ra[1][0], Ra[2][0]),
                       (Ra[0][1], Ra[1][1], Ra[2][1]),
                       (Ra[0][2], Ra[1][2], Ra[2][2]))
                Rr = ((R[0][0] * RaT[0][0] + R[0][1] * RaT[1][0] + R[0][2] * RaT[2][0],
                       R[0][0] * RaT[0][1] + R[0][1] * RaT[1][1] + R[0][2] * RaT[2][1],
                       R[0][0] * RaT[0][2] + R[0][1] * RaT[1][2] + R[0][2] * RaT[2][2]),
                      (R[1][0] * RaT[0][0] + R[1][1] * RaT[1][0] + R[1][2] * RaT[2][0],
                       R[1][0] * RaT[0][1] + R[1][1] * RaT[1][1] + R[1][2] * RaT[2][1],
                       R[1][0] * RaT[0][2] + R[1][1] * RaT[1][2] + R[1][2] * RaT[2][2]),
                      (R[2][0] * RaT[0][0] + R[2][1] * RaT[1][0] + R[2][2] * RaT[2][0],
                       R[2][0] * RaT[0][1] + R[2][1] * RaT[1][1] + R[2][2] * RaT[2][1],
                       R[2][0] * RaT[0][2] + R[2][1] * RaT[1][2] + R[2][2] * RaT[2][2]))
                # 平移：t_rel = t - Rr @ ta
                tr = (-(Rr[0][0] * ta[0] + Rr[0][1] * ta[1] + Rr[0][2] * ta[2]) + t[0],
                      -(Rr[1][0] * ta[0] + Rr[1][1] * ta[1] + Rr[1][2] * ta[2]) + t[1],
                      -(Rr[2][0] * ta[0] + Rr[2][1] * ta[1] + Rr[2][2] * ta[2]) + t[2])

                pts, has_intensity = parse_cloud(latest_cloud["msg"], Rr, tr)
                if pts:
                    world.add_points(pts, has_intensity)
                # 轨迹（约 5Hz 采样，避免轨迹点过密）
                if now - last_traj >= 0.2:
                    world.add_traj(tr)
                    last_traj = now

        if now - last_write > 1.0:
            payload = world.to_payload()
            payload["online"] = cloud_fresh and odom_fresh
            payload["written_at"] = now
            payload["anchored"] = anchor is not None
            if not payload["online"]:
                payload["reason"] = "cloud or odom missing/stale"
            tmp = CACHE_FILE + ".tmp"
            try:
                with open(tmp, "w") as f:
                    json.dump(payload, f, allow_nan=False)
                os.replace(tmp, CACHE_FILE)
            except (TypeError, ValueError) as e:
                print("[map_worker] dump error", e, flush=True)
            last_write = now


if __name__ == "__main__":
    main()
