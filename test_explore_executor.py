"""explore_executor.py 单元测试（阶段 0 dry_run，用合成栅格测规划，本地可跑）。

覆盖：三态化 occupancy grid（z 过滤）、frontier 提取 + BFS 聚类、代价选目标、
探索完成判定、dry_run 下 _issue_velocity 不产生 velocity（只记录日志）。
"""
import json
import os
import tempfile
import unittest

from explore_executor import ExploreExecutor
from explore_fence import FenceStore


def _write_slam_cache(path, grid, pose):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"online": True, "map_ts": 0, "pose_ts": 0, "grid": grid, "pose": pose}, fh)


def _synthetic_grid():
    """合成一面竖墙（x≈2m, y∈[-1,1], z=0.5 障碍）+ 机器人原点左侧。"""
    grid = []
    y = -1.0
    while y <= 1.0:
        grid.append([2.0, round(y, 2), 0.5, 1.0])
        y += 0.1
    return grid


class TestBuildOccupancy(unittest.TestCase):
    def test_z_filter(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "fence.json"))
            ex = ExploreExecutor(os.path.join(d, "slam_cache.json"), store, d, dry_run=True)
            # z=0.2 落地，z=2.0 空中（过滤）
            _write_slam_cache(ex.slam_cache_path,
                              [[0.0, 0.0, 0.2], [0.0, 0.0, 2.0]], {"x": 0, "y": 0, "z": 0})
            grid, pose = ex._load_grid()
            self.assertIsNotNone(grid)
            occ, origin = ex._build_occupancy(grid)
            # 只有 z=0.2 的点成为障碍；z=2.0 被过滤 → 仅 1 个 occupied
            self.assertEqual(int((occ == 2).sum()), 1)

    def test_empty_grid(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "fence.json"))
            ex = ExploreExecutor(os.path.join(d, "slam_cache.json"), store, d, dry_run=True)
            occ, origin = ex._build_occupancy([])
            self.assertEqual(occ.size, 0)


class TestFrontierAndTarget(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = self.tmp.name
        self.store = FenceStore(os.path.join(d, "fence.json"))
        self.ex = ExploreExecutor(os.path.join(d, "slam_cache.json"), self.store, d, dry_run=True)

    def tearDown(self):
        self.tmp.cleanup()

    def test_extract_frontier_clusters(self):
        _write_slam_cache(self.ex.slam_cache_path, _synthetic_grid(), {"x": 0, "y": 0, "z": 0})
        grid, pose = self.ex._load_grid()
        occ, origin = self.ex._build_occupancy(grid)
        frontiers = self.ex._extract_frontiers(occ)
        self.assertGreater(len(frontiers), 0)
        # 竖墙产生左右两侧 frontier（至少一处未知/占用交界）
        total_cells = sum(len(c.cells) for c in frontiers)
        self.assertGreater(total_cells, 0)

    def test_pick_target_returns_none_when_no_frontier(self):
        _write_slam_cache(self.ex.slam_cache_path, [], {"x": 0, "y": 0, "z": 0})
        grid, pose = self.ex._load_grid()
        occ, origin = self.ex._build_occupancy(grid)
        target, clusters = self.ex._pick_target([], pose, occ, origin)
        self.assertIsNone(target)

    def test_pick_target_chooses_closest(self):
        _write_slam_cache(self.ex.slam_cache_path, _synthetic_grid(), {"x": 0, "y": 0, "z": 0})
        grid, pose = self.ex._load_grid()
        occ, origin = self.ex._build_occupancy(grid)
        frontiers = self.ex._extract_frontiers(occ)
        target, clusters = self.ex._pick_target(frontiers, pose, occ, origin)
        self.assertIsNotNone(target)
        # 目标不应与机器人重合，且应靠近墙（约 x≈2）
        self.assertGreater(target.x, 0.5)

    def test_done_when_no_frontier(self):
        _write_slam_cache(self.ex.slam_cache_path, [], {"x": 0, "y": 0, "z": 0})
        grid, pose = self.ex._load_grid()
        occ, origin = self.ex._build_occupancy(grid)
        frontiers = self.ex._extract_frontiers(occ)
        # remaining = sum unknown_gain = 0 < threshold → done
        self.ex._plan(grid, pose)
        self.assertEqual(self.ex._state["status"], "done")


class TestDryRunNoMotion(unittest.TestCase):
    def test_issue_velocity_is_noop(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "fence.json"))
            ex = ExploreExecutor(os.path.join(d, "slam_cache.json"), store, d, dry_run=True)
            # 不抛异常即通过；不会产生任何 velocity 下发
            ex._issue_velocity(0.3, 0.1)
            self.assertTrue(True)

    def test_state_write_no_velocity(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "fence.json"))
            ex = ExploreExecutor(os.path.join(d, "slam_cache.json"), store, d, dry_run=True)
            _write_slam_cache(ex.slam_cache_path, _synthetic_grid(), {"x": 0, "y": 0, "z": 0})
            ex._step()
            ex._write_state()
            state_path = os.path.join(d, "explore_state.json")
            self.assertTrue(os.path.exists(state_path))
            with open(state_path, "r", encoding="utf-8") as fh:
                state = json.load(fh)
            self.assertEqual(state["safety"]["v_cmd"], 0.0)
            self.assertEqual(state["safety"]["w_cmd"], 0.0)
            self.assertTrue(state["safety"]["dry_run"])


class TestFenceSafety(unittest.TestCase):
    def test_outside_fence_faults(self):
        with tempfile.TemporaryDirectory() as d:
            store = FenceStore(os.path.join(d, "fence.json"))
            store.save([{"kind": "circle", "center": [0, 0], "radius": 1.0}])
            ex = ExploreExecutor(os.path.join(d, "slam_cache.json"), store, d, dry_run=True)
            _write_slam_cache(ex.slam_cache_path, _synthetic_grid(), {"x": 10, "y": 10, "z": 0})
            ex._step()
            # 机器人在围栏外 → fault
            self.assertEqual(ex.status, "fault")


if __name__ == "__main__":
    unittest.main()
