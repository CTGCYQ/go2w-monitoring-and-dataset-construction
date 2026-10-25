"""explore_fence.py 单元测试（本地可跑，无 ROS 依赖）。

覆盖：FenceStore 存取、validate 各分支（顶点数/面积/自交/矩形宽高/圆半径）、
point_in_fence 射线法 + margin 越界、圆判定。
"""
import os
import tempfile
import unittest

from explore_fence import (Fence, FenceStore, FenceValidationError, point_in_fence,
                           validate_fence, fences_contain, rectangle_to_polygon)


class TestFenceValidate(unittest.TestCase):
    def test_polygon_valid(self):
        fence = Fence("polygon", points=[(0, 0), (3, 0), (3, 3), (0, 3)])
        ok, msg = validate_fence(fence)
        self.assertTrue(ok, msg)

    def test_polygon_too_few_vertices(self):
        fence = Fence("polygon", points=[(0, 0), (1, 1)])
        ok, msg = validate_fence(fence)
        self.assertFalse(ok)
        self.assertIn("顶点", msg)

    def test_polygon_too_small_area(self):
        # 1x0.5 面积 0.5 < min_area 1
        fence = Fence("polygon", points=[(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)])
        ok, msg = validate_fence(fence)
        self.assertFalse(ok)
        self.assertIn("面积", msg)

    def test_self_intersecting_bowtie(self):
        # 蝴蝶结：边 (4,4)-(0,4) 与 (4,0)-(2,-1) 段交叉，面积为 2（非零）但自交
        fence = Fence("polygon", points=[(0, 0), (4, 4), (0, 4), (4, 0), (2, -1)])
        ok, msg = validate_fence(fence)
        self.assertFalse(ok)
        self.assertIn("自交", msg)

    def test_rectangle_valid(self):
        fence = Fence("rectangle", points=[(0, 0), (2, 2)])
        ok, _ = validate_fence(fence)
        self.assertTrue(ok)

    def test_rectangle_too_small_side(self):
        fence = Fence("rectangle", points=[(0, 0), (0.3, 5)])
        ok, _ = validate_fence(fence)
        self.assertFalse(ok)

    def test_circle_valid(self):
        fence = Fence("circle", center=(1, 1), radius=1.0)
        ok, _ = validate_fence(fence)
        self.assertTrue(ok)

    def test_circle_too_small_radius(self):
        fence = Fence("circle", center=(0, 0), radius=0.2)
        ok, _ = validate_fence(fence)
        self.assertFalse(ok)

    def test_validate_accepts_dict(self):
        ok, _ = validate_fence({"kind": "circle", "center": [0, 0], "radius": 0.5})
        self.assertTrue(ok)


class TestPointInFence(unittest.TestCase):
    def setUp(self):
        self.poly = Fence("polygon", points=[(0, 0), (4, 0), (4, 4), (0, 4)])

    def test_inside_center(self):
        self.assertTrue(point_in_fence(2, 2, self.poly))

    def test_outside(self):
        self.assertFalse(point_in_fence(5, 5, self.poly))
        self.assertFalse(point_in_fence(-1, 2, self.poly))

    def test_on_edge_outside_with_zero_margin(self):
        # 顶点/边上：射线法边界点判定为 inside（含）。
        self.assertTrue(point_in_fence(0, 2, self.poly, margin=0.0))

    def test_margin_violates_near_edge(self):
        # 距边 0.1 < margin 0.2 应判越界
        self.assertFalse(point_in_fence(0.1, 2, self.poly, margin=0.2))

    def test_concave_polygon(self):
        # 凹多边形：V 形缺口（(0,0)(4,0)(4,4)(2,2)(0,4) 的缺口在 (2,2) 附近）
        concave = Fence("polygon", points=[(0, 0), (4, 0), (4, 4), (2, 2), (0, 4)])
        self.assertTrue(point_in_fence(1, 1.5, concave))
        self.assertTrue(point_in_fence(3.5, 3.5, concave))
        self.assertFalse(point_in_fence(2, 2.5, concave))  # 缺口内

    def test_circle(self):
        circle = Fence("circle", center=(0, 0), radius=2.0)
        self.assertTrue(point_in_fence(1, 1, circle))
        self.assertFalse(point_in_fence(2.5, 0, circle))
        # margin 收缩
        self.assertFalse(point_in_fence(1.9, 0, circle, margin=0.2))


class TestRectangleToPolygon(unittest.TestCase):
    def test_expand(self):
        fence = Fence("rectangle", points=[(2, 5), (0, 1)])  # 对角任意顺序
        poly = rectangle_to_polygon(fence)
        self.assertEqual(poly, [(0, 1), (2, 1), (2, 5), (0, 5)])


class TestFenceStore(unittest.TestCase):
    def test_save_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "fence.json"))
            fences = [Fence("circle", center=(0, 0), radius=1.0),
                      Fence("polygon", points=[(0, 0), (2, 0), (2, 2), (0, 2)])]
            store.save(fences)
            loaded = store.load()
            self.assertEqual(len(loaded), 2)
            self.assertEqual(loaded[0].kind, "circle")
            self.assertEqual(loaded[1].kind, "polygon")

    def test_load_empty_missing(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "missing.json"))
            self.assertEqual(store.load(), [])

    def test_contains_static(self):
        fence = Fence("polygon", points=[(0, 0), (3, 0), (3, 3), (0, 3)])
        self.assertTrue(FenceStore.contains(fence, 1, 1))
        self.assertFalse(FenceStore.contains(fence, 4, 4))

    def test_fences_contain_union(self):
        f1 = Fence("circle", center=(0, 0), radius=1.0)
        f2 = Fence("circle", center=(10, 10), radius=1.0)
        self.assertTrue(fences_contain([f1, f2], 0.5, 0.5))
        self.assertTrue(fences_contain([f1, f2], 10.5, 10.5))
        self.assertFalse(fences_contain([f1, f2], 5, 5))
        # 无围栏：不约束
        self.assertTrue(fences_contain([], 100, 100))


if __name__ == "__main__":
    unittest.main()
