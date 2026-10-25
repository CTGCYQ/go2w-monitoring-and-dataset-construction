"""SLAM 自主探索 REST 路由（阶段 0 子集：只读/安全部分，不动 motion）。

阶段 0 实现：
  GET /api/explore/state        探索瞬时状态（dry_run 规划可视化）
  GET /api/explore/frontiers    当前前沿簇
  GET /api/explore/fence        读当前围栏
  PUT /api/explore/fence        保存/更新围栏（FenceStore.validate → 存，非法 422）
  GET /api/explore/jobs         探索作业列表（含 dry-run 作业记录）
  GET /api/explore/jobs/{jid}   单作业详情

阶段 0 不实现 arm/estop 真机运动（阶段 1 现场验收）；预留 stub 返回 501 或「不开放」。
挂载模式对齐 install_fleet / install_scenes：install_explore(app, BASE_DIR, ...)。
"""
import json
import time
import threading
import uuid
from pathlib import Path

from fastapi import HTTPException, Request

from explore_fence import FenceStore, FenceValidationError
from explore_executor import ExploreExecutor


def install_explore(app, root, slam_cache_path=None, robot_id="go2w-01", dry_run=True):
    root = Path(root)
    fence_store = FenceStore(root / "explore_fence.json")
    jobs_store = _JobsStore(root / "explore_jobs.json")
    executor = ExploreExecutor(
        slam_cache_path=Path(slam_cache_path) if slam_cache_path else root / "slam_cache.json",
        fence_store=fence_store, root_dir=root, robot_id=robot_id, dry_run=dry_run,
    )
    state_lock = threading.Lock()

    def _read_state():
        path = root / "explore_state.json"
        if not path.exists():
            return {
                "robot_id": robot_id, "online": False, "status": "idle",
                "pose": {"x": 0, "y": 0, "z": 0, "yaw": 0},
                "frontiers": [], "target": None,
                "safety": {"estop": False, "dry_run": dry_run, "fence_violation": False,
                           "obstacle_blocked": False, "fall_detected": False, "v_cmd": 0.0,
                           "w_cmd": 0.0, "heartbeat_ok": True},
                "progress": {"total_frontier": 0, "remaining_frontier": 0, "covered_cells": 0},
                "ts": time.time(),
            }
        try:
            return json.loads(path.read_text("utf-8"))
        except (OSError, ValueError):
            return {"robot_id": robot_id, "online": False, "status": "idle",
                    "pose": {"x": 0, "y": 0, "z": 0, "yaw": 0}, "frontiers": [], "target": None,
                    "safety": {"estop": False, "dry_run": dry_run, "fence_violation": False,
                               "obstacle_blocked": False, "fall_detected": False, "v_cmd": 0.0,
                               "w_cmd": 0.0, "heartbeat_ok": True},
                    "progress": {"total_frontier": 0, "remaining_frontier": 0, "covered_cells": 0},
                    "ts": time.time()}

    @app.on_event("startup")
    def explore_start():
        executor.start()

    @app.on_event("shutdown")
    def explore_stop():
        executor.close()

    @app.get("/api/explore/state")
    def explore_state():
        state = _read_state()
        state.setdefault("safety", {}).setdefault("dry_run", dry_run)
        return state

    @app.get("/api/explore/frontiers")
    def explore_frontiers():
        state = _read_state()
        return {
            "frontiers": state.get("frontiers", []),
            "threshold": executor.params.get("frontier_threshold"),
            "status": state.get("status"),
        }

    @app.get("/api/explore/fence")
    def explore_get_fence():
        return {"fences": [f.to_dict() for f in fence_store.load()]}

    @app.put("/api/explore/fence")
    async def explore_put_fence(request: Request):
        try:
            body = await request.json()
        except ValueError:
            raise HTTPException(422, "请求必须是 JSON")
        fences = body.get("fences") if isinstance(body, dict) else None
        if fences is None and isinstance(body, list):
            fences = body
        if not isinstance(fences, list):
            raise HTTPException(422, "请求须包含 fences 数组")
        # 逐条校验，任一非法 422
        validated = []
        for item in fences:
            ok, msg = fence_store.validate(item)
            if not ok:
                raise HTTPException(422, "围栏校验失败：" + msg)
            validated.append(item)
        fence_store.save(validated)
        return {"ok": True, "fences": [f.to_dict() for f in validated]}

    @app.get("/api/explore/jobs")
    def explore_jobs():
        jobs = jobs_store.list()
        return {"jobs": jobs, "count": len(jobs), "dry_run": dry_run}

    @app.get("/api/explore/jobs/{jid}")
    def explore_job(jid: str):
        job = jobs_store.get(jid)
        if job is None:
            raise HTTPException(404, "探索作业不存在")
        return job

    # ---- 阶段 0 不开放：真机运动相关端点，预留 stub ----
    @app.post("/api/explore/estop", status_code=501)
    async def explore_estop():
        raise HTTPException(501, "阶段 0 不开放：真机急停需现场验收（阶段 1）")

    @app.post("/api/explore/jobs/{jid}/arm", status_code=501)
    async def explore_arm(jid: str):
        raise HTTPException(501, "阶段 0 不开放：开始探索真机放行需现场验收（阶段 1）")

    @app.post("/api/explore/jobs/{jid}/pause", status_code=501)
    async def explore_pause(jid: str):
        raise HTTPException(501, "阶段 0 不开放")

    @app.post("/api/explore/jobs/{jid}/resume", status_code=501)
    async def explore_resume(jid: str):
        raise HTTPException(501, "阶段 0 不开放")

    @app.post("/api/explore/jobs/{jid}/cancel", status_code=501)
    async def explore_cancel(jid: str):
        raise HTTPException(501, "阶段 0 不开放")

    app.state.explore_executor = executor
    app.state.explore_fence_store = fence_store
    app.state.explore_jobs_store = jobs_store
    return executor


class _JobsStore:
    """探索作业持久化（阶段 0 仅用于 dry-run 作业记录，不驱动真机）。"""

    def __init__(self, path):
        self.path = str(path)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def list(self):
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
        except (OSError, ValueError):
            return []
        return raw if isinstance(raw, list) else []

    def get(self, jid):
        for job in self.list():
            if job.get("id") == jid:
                return job
        return None

    def create(self, value):
        job = {
            "id": uuid.uuid4().hex,
            "robot_id": value.get("robot_id", "go2w-01"),
            "status": "queued",
            "dry_run": True,
            "created_at": time.time(),
        }
        with self._lock:
            jobs = self.list()
            jobs.insert(0, job)
            self._write(jobs)
        return job

    def _write(self, jobs):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(jobs, fh, ensure_ascii=False)
        import os
        os.replace(tmp, self.path)
