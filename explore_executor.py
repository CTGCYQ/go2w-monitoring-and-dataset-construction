"""SLAM 自主探索执行器（阶段 0：仿真/规划可视化，dry_run，绝不下发运动指令）。

读 slam_cache.json（grid + pose，只读），三态化 occupancy grid（occupied/free/unknown，
z∈[-0.15,0.8] 过滤落地障碍，0.1m/cell），frontier 提取 + BFS 聚类 + 代价选目标，
探索完成判定（remaining_frontier < threshold），写独立 explore_state.json。

安全边界（不可违反）：
  1. 阶段 0 以 dry_run=True 运行，_issue_velocity() 只写日志，不调用任何 motion/text_service。
  2. 绝不 touch motion_enabled / robot.yaml / motion-gate。
  3. 只读 slam_cache.json，写独立 explore_state.json / explore_jobs.json。

状态机 C1：idle→arming→armed→exploring→paused→done/fault；阶段 0 只走 dry_run 路径，
不提供 arm/estop 真机放行（阶段 1 真机运动相关，另行实现）。
"""
import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

from explore_fence import FenceStore, fences_contain

# 三态 occupancy grid 取值
UNKNOWN = 0
FREE = 1
OCCUPIED = 2

# 落地障碍 z 过滤窗口与栅格分辨率
Z_MIN = -0.15
Z_MAX = 0.8
CELL = 0.1

# 代价函数默认权重
W1 = 1.0   # 距离代价（鼓励近）
W2 = 0.4   # 信息增益未知数（鼓励未知多的方向）
W3 = 0.3   # 去重（近期去过则代价高）

# 探索完成判定默认阈值：剩余有效前沿 cell 数（或前端可配）
FRONTIER_THRESHOLD = 20

STATUS_SET = {"idle", "arming", "armed", "exploring", "paused", "done", "fault"}


class ExploreTarget:
    """候选探索目标点。"""

    def __init__(self, x, y, cells, unknown_gain, seq):
        self.x, self.y = x, y
        self.cells = cells
        self.unknown_gain = unknown_gain
        self.seq = seq

    def to_dict(self):
        return {"x": round(self.x, 3), "y": round(self.y, 3), "cells": self.cells,
                "unknown_gain": round(self.unknown_gain, 2), "seq": self.seq}


class FrontierCluster:
    """BFS 聚出的前沿簇。"""

    def __init__(self, cells):
        self.cells = cells
        # 质心
        n = len(cells)
        self.x = sum(c[0] for c in cells) / n
        self.y = sum(c[1] for c in cells) / n
        # unknown 邻居数（信息增益）
        self.unknown_gain = n

    def to_dict(self):
        return {"x": round(self.x, 3), "y": round(self.y, 3), "cells": len(self.cells)}


class ExploreExecutor:
    def __init__(self, slam_cache_path, fence_store, root_dir, dry_run=True,
                 robot_id="go2w-01", config=None):
        self.slam_cache_path = Path(slam_cache_path)
        self.fence_store = fence_store
        self.root_dir = Path(root_dir)
        self.robot_id = robot_id
        self.dry_run = dry_run
        cfg = config or {}
        self.params = {
            "v_max_explore": float(cfg.get("v_max_explore", 0.3)),
            "fence_margin": float(cfg.get("fence_margin", 0.2)),
            "min_dist_obstacle": float(cfg.get("min_dist_obstacle", 0.6)),
            "frontier_threshold": int(cfg.get("frontier_threshold", FRONTIER_THRESHOLD)),
            "cell": float(cfg.get("cell", CELL)),
            "w1": float(cfg.get("w1", W1)),
            "w2": float(cfg.get("w2", W2)),
            "w3": float(cfg.get("w3", W3)),
            "z_min": float(cfg.get("z_min", Z_MIN)),
            "z_max": float(cfg.get("z_max", Z_MAX)),
        }
        # 状态机
        self.status = "idle"
        self.explore_armed = False
        self.estop = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        # 去重：近期访问过的目标位置
        self.recent_visit = {}
        # 兼容 design 签名 state_fn/obstacle_fn/motion_gate_fn（阶段 1 真机用）
        self.state_fn = None
        self.obstacle_fn = None
        self.motion_gate_fn = None
        # 每步产出
        self._state = None

    # ---------- 生命周期 ----------
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="explore-executor", daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    def arm(self):
        """设置 explore_armed=true（阶段 1 真机需配合 motion_enabled）。阶段 0 恒 dry_run，不产生运动。"""
        with self._lock:
            self.explore_armed = True
            if self.status in {"idle", "arming"}:
                self.status = "armed" if self.dry_run else "arming"

    def disarm(self, reason=""):
        with self._lock:
            self.explore_armed = False
            if self.status not in {"idle", "done", "fault"}:
                self.status = "idle"

    def estop(self):
        """急停：阶段 0 只置 estop=true + disarm，绝不下发 0 速（本就不下发）。"""
        with self._lock:
            self.estop = True
            self.explore_armed = False
            self.status = "paused"

    def pause(self):
        with self._lock:
            if self.status == "exploring":
                self.status = "paused"

    def resume(self):
        with self._lock:
            if self.status == "paused" and not self.estop:
                self.status = "exploring"

    # ---------- 主循环（0.5s 周期，dry_run）----------
    def _loop(self):
        while not self._stop.is_set():
            try:
                self._step()
                self._write_state()
            except Exception:
                # 阶段 0 不因异常退出线程，避免前端拉空状态
                pass
            self._stop.wait(0.5)

    def _transition(self, status):
        with self._lock:
            self.status = status

    def _step(self):
        """perceive→safety→plan→命令（dry_run 命令为 no-op）。"""
        grid, pose = self._load_grid()
        if grid is None or pose is None:
            self._transition("idle")
            return
        # 安全校验：围栏越界（有围栏时判定）
        fences = self.fence_store.load()
        if fences and not fences_contain(fences, pose["x"], pose["y"], self.params["fence_margin"]):
            self._transition("fault")
            return
        if self.dry_run:
            # 阶段 0 只做规划可视化，不进入 armed/exploring 真机运动；仍展示 idle/armed 规划结果
            self._plan(grid, pose)
            if self.status not in {"armed", "exploring", "done"}:
                self._transition("armed")
            return
        # 阶段 1+ 真机运动路径（本文件阶段 0 不实现，保留占位）
        self._transition("armed")

    def _plan(self, grid, pose):
        """按网格三态化 → frontier 提取 → BFS 聚类 → 代价选目标。产出写 _state。"""
        occ, origin = self._build_occupancy(grid)
        frontiers = self._extract_frontiers(occ)
        target, clusters = self._pick_target(frontiers, pose, occ, origin)
        remaining = sum(len(c.cells) for c in frontiers)  # 有效前沿 cell 数（探索完成判定用）
        done = (len(frontiers) == 0) or (remaining < self.params["frontier_threshold"])
        self._state = {
            "robot_id": self.robot_id,
            "online": True,
            "status": "done" if done else ("exploring" if self.explore_armed else "armed"),
            "pose": pose,
            "frontiers": [c.to_dict() for c in clusters],
            "target": target.to_dict() if target else None,
            "safety": {
                "estop": bool(self.estop),
                "fence_violation": False,
                "obstacle_blocked": False,
                "fall_detected": False,
                "v_cmd": 0.0,
                "w_cmd": 0.0,
                "heartbeat_ok": True,
                "dry_run": bool(self.dry_run),
            },
            "progress": {
                "total_frontier": len(clusters),
                "remaining_frontier": remaining,
                "covered_cells": int((occ == FREE).sum() + (occ == OCCUPIED).sum()),
                "target_index": target.seq if target else None,
            },
            "ts": time.time(),
        }
        if done:
            self._transition("done")

    # ---------- 数据读入：slam_cache.json（只读）----------
    def _load_grid(self):
        """读 slam_cache.json 的 grid + pose。返回 (grid, pose) 或 (None, None)。"""
        if not self.slam_cache_path.exists():
            return None, None
        try:
            data = json.loads(self.slam_cache_path.read_text("utf-8"))
        except (OSError, ValueError):
            return None, None
        if not isinstance(data, dict):
            return None, None
        grid = data.get("grid") or []
        pose = data.get("pose")
        if not grid or not isinstance(pose, dict):
            return None, None
        return grid, {"x": float(pose.get("x", 0)), "y": float(pose.get("y", 0)),
                      "z": float(pose.get("z", 0)), "yaw": float(pose.get("yaw", 0.0))}

    # ---------- 三态化 occupancy grid ----------
    def _build_occupancy(self, grid):
        """把 slam_cache.grid 投影到 2D，三态化到固定 cell。

        返回 (occ, origin)。occ 形状 (h, w)，值 ∈ {UNKNOWN, FREE, OCCUPIED}。
        只取 z ∈ [z_min, z_max] 的点作为落地障碍；空区域 free，其余 unknown。
        """
        pts = []
        for p in (grid or []):
            if not isinstance(p, (list, tuple)) or len(p) < 3:
                continue
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if self.params["z_min"] <= z <= self.params["z_max"]:
                pts.append((x, y))
        if not pts:
            # 无有效障碍点：返回空栅格，写入状态用
            self._grid_meta = {"origin": (0.0, 0.0), "w": 0, "h": 0}
            occ = _empty_grid()
            return occ, (0.0, 0.0)
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        cell = self.params["cell"]
        # 外扩一圈，容纳 margin
        x0 = minx - cell; y0 = miny - cell
        w = int(math.ceil((maxx - minx) / cell)) + 2
        h = int(math.ceil((maxy - miny) / cell)) + 2
        occ = _filled(UNKNOWN, h, w)
        for x, y in pts:
            ci = int((x - x0) / cell)
            rj = int((y - y0) / cell)
            if 0 <= ci < w and 0 <= rj < h:
                occ[rj, ci] = OCCUPIED
        self._grid_meta = {"origin": (x0, y0), "w": w, "h": h, "cell": cell}
        return occ, (x0, y0)

    # ---------- frontier 提取 + BFS 聚类 ----------
    def _extract_frontiers(self, occ):
        """frontier cell = FREE 且 8 邻域至少 1 个 UNKNOWN。用 BFS 聚成 cluster。

        注意：阶段 0 的 occ 只有 UNKNOWN/OCCUPIED 两态（无 free 观测），
        因此 frontier 定义为「OCCUPIED 边界的相邻 UNKNOWN 侧」，即未知与已建图交界。
        """
        h, w = occ.shape
        # 标记 frontier：unknown 且周围有 occupied（已建图边界）
        front = _filled(False, h, w)
        for r in range(h):
            for c in range(w):
                if occ[r, c] != UNKNOWN:
                    continue
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < h and 0 <= nc < w and occ[nr, nc] == OCCUPIED:
                            front[r, c] = True
                            break
                    if front[r, c]:
                        break
        # BFS 聚类
        seen = _filled(False, h, w)
        clusters = []
        for r in range(h):
            for c in range(w):
                if front[r, c] and not seen[r, c]:
                    cells = []
                    q = deque([(r, c)])
                    seen[r, c] = True
                    while q:
                        cr, cc = q.popleft()
                        cells.append((cr, cc))
                        for dr in (-1, 0, 1):
                            for dc in (-1, 0, 1):
                                if dr == 0 and dc == 0:
                                    continue
                                nr, nc = cr + dr, cc + dc
                                if 0 <= nr < h and 0 <= nc < w and front[nr, nc] and not seen[nr, nc]:
                                    seen[nr, nc] = True
                                    q.append((nr, nc))
                    clusters.append(cells)
        return [FrontierCluster(cs) for cs in clusters]

    # ---------- 代价选目标 ----------
    def _pick_target(self, frontiers, pose, occ, origin):
        """对每个 cluster 取质心为候选，代价 w1·dist + w2/unknown_gain + w3·inv(recent_visit)。"""
        if not frontiers:
            return None, []
        px, py = pose["x"], pose["y"]
        cell = self.params["cell"]
        x0, y0 = origin
        w1, w2, w3 = self.params["w1"], self.params["w2"], self.params["w3"]
        candidates = []
        for idx, cl in enumerate(frontiers):
            # 二维网格坐标 → 世界坐标
            wx = x0 + cl.x * cell
            wy = y0 + cl.y * cell
            dist = math.hypot(wx - px, wy - py)
            unknown_gain = max(1, len(cl.cells))
            # 近期访问：visit 键为量化到 0.5m 的网格，近期去过则代价高
            key = (round(wx * 2), round(wy * 2))
            recent = self.recent_visit.get(key, 0.0)
            cost = w1 * dist + w2 / unknown_gain + w3 / (1.0 + recent)
            candidates.append((cost, dist, unknown_gain, wx, wy, idx, key))
        candidates.sort(key=lambda t: t[0])
        cost, dist, unknown_gain, wx, wy, idx, key = candidates[0]
        target = ExploreTarget(wx, wy, len(frontiers[idx].cells), unknown_gain, idx)
        # 记录近期访问（去重回路）
        self.recent_visit[key] = 1.0
        for k in list(self.recent_visit):
            self.recent_visit[k] *= 0.9  # 衰减
            if self.recent_visit[k] < 0.05:
                del self.recent_visit[k]
        return target, frontiers

    # ---------- 安全校验（阶段 0 只做围栏，其余留阶段 1）----------
    def _safety_check(self, pose):
        """阶段 0 的简化安全汇总：只做围栏越界（dry_run），L3~L7 阶段 1 实现。"""
        fences = self.fence_store.load()
        if fences and not fences_contain(fences, pose["x"], pose["y"], self.params["fence_margin"]):
            return {"ok": False, "fence_violation": True, "reason": "robot outside fence"}
        return {"ok": True, "fence_violation": False}

    # ---------- 运动指令下发（阶段 0 no-op）----------
    def _issue_velocity(self, v, w):
        """阶段 0：dry_run 下为 no-op，只写日志，绝不调用 motion/text_service。"""
        if self.dry_run:
            self._log(f"[dry_run] issue_velocity v={v:.3f} w={w:.3f} (no-op, not sent)")
            return
        # 阶段 1+ 真机：此处接入 text_service 白名单通道（方案 A move_distance/turn_angle）
        # 且必须前置 motion_enabled + explore_armed 双门 + 围栏/避障预检。
        raise NotImplementedError("stage 1 motion not implemented")

    def _log(self, msg):
        print(f"[explore:{self.robot_id}] {msg}", flush=True)

    # ---------- 状态持久化 ----------
    def _write_state(self):
        path = self.root_dir / "explore_state.json"
        state = self._state or {
            "robot_id": self.robot_id, "online": False, "status": self.status,
            "pose": {"x": 0, "y": 0, "z": 0, "yaw": 0},
            "frontiers": [], "target": None,
            "safety": {"estop": bool(self.estop), "dry_run": bool(self.dry_run),
                       "fence_violation": False, "obstacle_blocked": False,
                       "fall_detected": False, "v_cmd": 0.0, "w_cmd": 0.0, "heartbeat_ok": True},
            "progress": {"total_frontier": 0, "remaining_frontier": 0, "covered_cells": 0},
            "ts": time.time(),
        }
        try:
            with self._lock:
                tmp = str(path) + ".tmp"
                with open(tmp, "w", encoding="utf-8") as fh:
                    json.dump(state, fh, ensure_ascii=False)
                os.replace(tmp, path)
        except OSError:
            pass


def _filled(val, h, w):
    import numpy as np
    return np.full((h, w), val, dtype=np.int8)


def _empty_grid():
    import numpy as np
    return np.zeros((0, 0), dtype=np.int8)
