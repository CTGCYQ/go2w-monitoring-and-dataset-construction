"""探索围栏模型 + 合法性校验 + 越界判定（纯几何，无 ROS，可本地单测）。

围栏是探索的「安全硬约束」，坐标与 slam_cache.grid 同 world_map 参考系。
kind ∈ polygon/rectangle/circle；rectangle 存 2 个对角点，circle 存 center+radius。
越界判定：多边形用射线法 point-in-polygon + 点到边距离 ≤ margin 即越界；圆用
dist(center) > radius - margin。validate 做面积/自交/最小尺寸预校验，避免退化围栏。
"""
import json
import os
import threading
import time
from pathlib import Path

MIN_AREA = 1.0          # 最小面积 m^2
MIN_RECT_SIDE = 0.5     # 矩形最小边长 m
MIN_RADIUS = 0.3        # 圆最小半径 m


class FenceValidationError(ValueError):
    """围栏几何不合法（自交/面积不足/顶点不足等），前端与后端双保险共用。"""


def _cross(o, a, b):
    """向量 OA 与 OB 的叉积 z 分量。"""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _seg_seg_intersect(a, b, c, d):
    """线段 ab 与 cd 是否相交（含端点接触）。"""
    def side(p, q, r):
        v = _cross(p, q, r)
        return (v > 0) - (v < 0)
    s1 = side(a, b, c); s2 = side(a, b, d); s3 = side(c, d, a); s4 = side(c, d, b)
    if s1 != s2 and s3 != s4:
        return True
    # 共线端点接触：任一端点落在对方线段上即视为接触
    if s1 == 0 and _on_segment(a, b, c):
        return True
    if s2 == 0 and _on_segment(a, b, d):
        return True
    if s3 == 0 and _on_segment(c, d, a):
        return True
    if s4 == 0 and _on_segment(c, d, b):
        return True
    return False


def _on_segment(a, b, p, eps=1e-9):
    """点 p 是否在端点含端点的线段 ab 上。"""
    if abs(_cross(a, b, p)) > eps:
        return False
    return min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps \
        and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps


def _point_seg_distance(p, a, b):
    """点到线段 ab 的欧氏距离。"""
    px, py = p; ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5


def _polygon_area(points):
    """Shoelace 多边形有向面积（闭合多边形组）。返回带符号值，逆时针为正。"""
    n = len(points)
    s = 0.0
    for i in range(n):
        x1, y1 = points[i]; x2, y2 = points[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def _polygon_self_intersect(points):
    """非相邻边是否相交（多边形不自交判定）。"""
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or j == (i + 1) % n:
                continue
            c, d = points[j], points[(j + 1) % n]
            if _seg_seg_intersect(a, b, c, d):
                return True
    return False


class Fence:
    """一条围栏：kind ∈ polygon/rectangle/circle。points/center/radius 均为地图坐标。"""

    def __init__(self, kind, points=None, center=None, radius=None, fence_id=None, frame="world_map"):
        self.kind = kind
        self.points = [tuple(map(float, p)) for p in (points or [])]
        self.center = tuple(map(float, center)) if center else None
        self.radius = float(radius) if radius is not None else None
        self.fence_id = fence_id
        self.frame = frame

    def to_dict(self):
        d = {"kind": self.kind, "frame": self.frame}
        if self.kind == "circle":
            d["center"] = list(self.center) if self.center else None
            d["radius"] = self.radius
        else:
            d["points"] = [list(p) for p in self.points]
        if self.fence_id:
            d["fence_id"] = self.fence_id
        return d

    @staticmethod
    def from_dict(value):
        if not isinstance(value, dict):
            raise FenceValidationError("围栏必须是对象")
        kind = value.get("kind")
        if kind not in {"polygon", "rectangle", "circle"}:
            raise FenceValidationError("围栏类型须为 polygon/rectangle/circle")
        if kind == "circle":
            c = value.get("center")
            if not isinstance(c, (list, tuple)) or len(c) != 2 or value.get("radius") is None:
                raise FenceValidationError("圆形围栏需要 center 与 radius")
            return Fence(kind, center=c, radius=value["radius"],
                         fence_id=value.get("fence_id"), frame=value.get("frame", "world_map"))
        pts = value.get("points")
        if not isinstance(pts, (list, tuple)) or len(pts) < 2:
            raise FenceValidationError("多边形围栏需要 points 顶点")
        return Fence(kind, points=[tuple(p) for p in pts],
                     fence_id=value.get("fence_id"), frame=value.get("frame", "world_map"))


def point_in_fence(px, py, fence, margin=0.0):
    """越界判定：返回 True 表示在围栏内（含仅略微越界但仍安全）。

    margin 含义为「安全余量」：距边界距离 ≤ margin 的点视为已越界（返回 False）。
    多边形：射线法判定内点 + 对各边计算点到线段距离 ≤ margin 即越界。
    圆：dist(center) > radius - margin 即越界。
    """
    if fence.kind == "circle":
        if fence.center is None or fence.radius is None:
            return False
        cx, cy = fence.center
        return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5 <= fence.radius - margin

    pts = fence.points
    if len(pts) < 3:
        return False
    # 射线法 point-in-polygon
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]; xj, yj = pts[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    if not inside:
        return False
    # 点到每条边的距离 ≤ margin 则视为越界（逼近/压边即停，防止压线）
    if margin > 0:
        for i in range(len(pts)):
            a = pts[i]; b = pts[(i + 1) % len(pts)]
            if _point_seg_distance((px, py), a, b) <= margin:
                return False
    return True


def validate_fence(fence, min_area=MIN_AREA):
    """围栏合法性校验，返回 (合法, 错误信息或空串)。"""
    if not isinstance(fence, Fence):
        try:
            fence = Fence.from_dict(fence)
        except FenceValidationError as e:
            return False, str(e)
    if fence.kind == "circle":
        r = fence.radius
        if r is None or r < MIN_RADIUS:
            return False, f"圆形围栏半径须 ≥ {MIN_RADIUS}m"
        return True, ""
    pts = fence.points
    if fence.kind == "polygon":
        if len(pts) < 3:
            return False, "多边形围栏顶点数须 ≥ 3"
        if abs(_polygon_area(pts)) < min_area:
            return False, f"围栏面积须 ≥ {min_area}m²"
        if _polygon_self_intersect(pts):
            return False, "围栏边线存在自交"
        return True, ""
    # rectangle：存 2 对角点
    if len(pts) != 2:
        return False, "矩形围栏需要 2 个对角点"
    x1, y1 = pts[0]; x2, y2 = pts[1]
    w, h = abs(x2 - x1), abs(y2 - y1)
    if w < MIN_RECT_SIDE or h < MIN_RECT_SIDE:
        return False, f"矩形围栏宽高须 ≥ {MIN_RECT_SIDE}m"
    if w * h < min_area:
        return False, f"围栏面积须 ≥ {min_area}m²"
    return True, ""


def rectangle_to_polygon(fence):
    """将矩形围栏展开为 4 点多边形（用于绘制/射线法）。"""
    if fence.kind != "rectangle" or len(fence.points) != 2:
        return fence.points
    (x1, y1), (x2, y2) = fence.points
    xmin, xmax = min(x1, x2), max(x1, x2)
    ymin, ymax = min(y1, y2), max(y1, y2)
    return [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]


class FenceStore:
    """围栏持久化：load/save/validate/contains。文件锁保证并发安全。"""

    def __init__(self, path):
        self.path = str(path)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def load(self):
        if not os.path.exists(self.path):
            return []
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
        except (OSError, ValueError):
            return []
        fences = []
        for item in (raw if isinstance(raw, list) else []):
            try:
                fences.append(Fence.from_dict(item))
            except FenceValidationError:
                continue
        return fences

    def save(self, fences):
        # 归一化：允许传入 dict 围栏，统一从 from_dict 转 Fence 再序列化
        normalized = []
        for f in fences:
            normalized.append(f if isinstance(f, Fence) else Fence.from_dict(f))
        data = [f.to_dict() for f in normalized]
        with self._lock:
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(data, fh, ensure_ascii=False)
            os.replace(tmp, self.path)

    def validate(self, fence):
        return validate_fence(fence)

    @staticmethod
    def contains(fence, x, y, margin=0.0):
        return point_in_fence(x, y, fence, margin)


def fences_contain(fences, x, y, margin=0.0):
    """多围栏判定：任一圈住即视为在围栏内（union 语义）。无围栏时返回 True（未约束）。"""
    if not fences:
        return True
    return any(point_in_fence(x, y, f, margin) for f in fences)
