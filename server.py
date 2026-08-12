"""Go2-W 机器人遥测 Web 后端（FastAPI）。

对外提供一套 REST API，供前端页面和外部程序调用，主要职责：
1. 实时状态：读取 collector 写入的 state.json，返回最新机器人状态/健康度；
2. 历史查询：代理查询 InfluxDB，返回电压/IMU/电池等历史曲线；
3. 录制控制：通过 control.json 向 collector 下发开始/停止录制命令；
4. 定时规则：管理 schedule.json 中的自动录制规则（增删查）；
5. 会话管理：列出已录制的 session 及元信息；
6. 数据集导出：把 session 导出为 HuggingFace 或 LeRobot 训练格式；
7. 帧浏览与标注：读取 session 的 raw.arrow 展示帧摘要，支持自然语言标注 CRUD；
8. 图片代理：代理返回 session 目录下的相机 JPEG 帧（浏览器不能直接访问文件系统）。

启动方式（服务器上）：
    python server.py            # 监听 0.0.0.0:8000
"""

from __future__ import annotations

import json    # 会话元信息 / 标注 JSON 读写
import time    # 轮询等待 collector 确认命令
import uuid    # 生成标注 ID
from pathlib import Path
from typing import Optional

import requests  # 代理查询 InfluxDB
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

from control_schema import read_control, set_command, write_control, DEFAULT_CONTROL
from dataset_builder import build_session_dataset, build_lerobot_dataset
from scheduler import ScheduleManager, ScheduleRule

# ---------------------------------------------------------------------------
# 全局配置
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent   # 项目部署目录
STATE_FILE = BASE_DIR / "state.json"         # collector 写的最新状态
CONTROL_FILE = BASE_DIR / "control.json"     # 录制控制命令文件
WEB_DIR = BASE_DIR / "web"                   # 前端静态资源目录

INFLUX_URL = "http://127.0.0.1:8091"         # InfluxDB HTTP 端口
DATABASE = "go2w_monitor"                    # InfluxDB 数据库名
DATASET_ROOT = "/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-dataset"  # 数据集根目录
SCHEDULE_FILE = BASE_DIR / "schedule.json"   # 定时规则文件

app = FastAPI(title="Go2-W Monitor", version="2.0.0")
_scheduler = ScheduleManager(str(SCHEDULE_FILE))


def read_state() -> dict:
    """读取 collector 写入的最新状态快照（不存在/损坏时返回安全默认）。"""
    if not STATE_FILE.exists():
        return {"online": False, "reason": "collector not running"}
    try:
        return json.loads(STATE_FILE.read_text("utf-8"))
    except Exception:
        return {"online": False, "reason": "state unreadable"}


def influx_query(q: str) -> Optional[list]:
    """执行 InfluxDB 查询，返回 series 列表；失败返回 None。"""
    try:
        r = requests.get(
            f"{INFLUX_URL}/query",
            params={"db": DATABASE, "q": q, "epoch": "ms"},
            timeout=5,
        )
        if r.status_code == 200:
            data = r.json()
            series = data.get("results", [{}])[0].get("series", [])
            return series
    except Exception:
        pass
    return None


@app.get("/api/state")
def api_state():
    """返回机器人最新状态快照（供前端 1 秒轮询）。"""
    st = read_state()
    return JSONResponse(st)


@app.get("/api/health")
def api_health():
    """健康检查：服务器状态 + 机器人在线状态 + 采集频率。"""
    st = read_state()
    online = st.get("online", False)
    return {
        "server": "ok",
        "robot_online": online,
        "hz": st.get("hz", 0),
        "last_seen_s": st.get("last_seen_s"),
        "tick": st.get("tick", 0),
    }


@app.get("/api/history/{measurement}")
def api_history(measurement: str, minutes: int = 5, limit: int = 500):
    """查询某类测量数据的历史曲线（白名单校验防注入）。"""
    valid = {"go2w_power", "go2w_imu", "go2w_imu_rpy", "go2w_battery",
             "go2w_motor", "go2w_foot_force"}
    if measurement not in valid:
        raise HTTPException(400, f"unknown measurement {measurement}")
    minutes = max(1, min(minutes, 60))
    limit = max(10, min(limit, 5000))
    q = (
        f"SELECT * FROM \"{measurement}\" WHERE time > now() - {minutes}m "
        f"LIMIT {limit}"
    )
    series = influx_query(q)
    if not series:
        return {"measurement": measurement, "points": []}
    rows = []
    for s in series:
        cols = s.get("columns", [])
        for vals in s.get("values", []):
            row = dict(zip(cols, vals))
            rows.append(row)
    return {"measurement": measurement, "points": rows}


@app.get("/api/checkpoints")
def api_checkpoints():
    # placeholder; future: list collected state snapshots
    return {"count": 0}


# ---------------------------------------------------------------------------
# 录制控制：通过 control.json 与 collector 通信
# ---------------------------------------------------------------------------

def _collector_status() -> dict:
    """汇总录制状态 = control.json（命令状态）+ state.json（实时录制进度）。"""
    ctrl = read_control(CONTROL_FILE)
    st = read_state()
    rec = ctrl.get("recording", DEFAULT_CONTROL["recording"])
    out = {
        "active": rec.get("active", False),
        "session_id": rec.get("session_id"),
        "start_ts": rec.get("start_ts"),
        "sample_count": rec.get("sample_count", 0),
        "size_bytes": rec.get("size_bytes", 0),
        "error": rec.get("error"),
        "pending_command": ctrl.get("command"),
        "robot_online": st.get("online", False),
    }
    # 用 state.json 中的实时进度覆盖（collector 每秒更新一次）
    live = st.get("recording")
    if live:
        out.update({
            "active": live.get("active", out["active"]),
            "session_id": live.get("session_id", out["session_id"]),
            "start_ts": live.get("start_ts", out["start_ts"]),
            "sample_count": live.get("sample_count", out["sample_count"]),
            "size_bytes": live.get("size_bytes", out["size_bytes"]),
        })
    return out


@app.get("/api/record/status")
def api_record_status():
    """返回当前录制状态。"""
    return JSONResponse(_collector_status())


@app.post("/api/record/start")
def api_record_start(label: str = ""):
    """下发开始录制命令并等待 collector 确认（最多 6 秒）。

    前置校验：不能重复录制、不能有未处理完的命令、机器人必须在线。
    """
    ctrl = read_control(CONTROL_FILE)
    if ctrl.get("recording", {}).get("active"):
        raise HTTPException(409, "recording already active")
    if ctrl.get("command") is not None:
        raise HTTPException(409, f"command {ctrl['command']} still pending")
    st = read_state()
    if not st.get("online"):
        raise HTTPException(400, "robot offline; cannot start recording")
    command_id = uuid.uuid4().hex[:12]
    set_command(CONTROL_FILE, "start", command_id, label=label)
    # 轮询等待 collector 处理（命令清空 + active 变 true）
    for _ in range(30):
        time.sleep(0.2)
        now = read_control(CONTROL_FILE)
        if now.get("command") is None and now.get("recording", {}).get("active"):
            return JSONResponse({"status": "recording",
                                 "session_id": now["recording"]["session_id"]})
    return JSONResponse({"status": "timeout", "reason": "collector not responding"}, status_code=503)


@app.post("/api/record/stop")
def api_record_stop():
    """下发停止录制命令并等待 collector 完成落盘（最多 30 秒）。"""
    ctrl = read_control(CONTROL_FILE)
    if not ctrl.get("recording", {}).get("active"):
        raise HTTPException(400, "no active recording")
    if ctrl.get("command") is not None:
        raise HTTPException(409, f"command {ctrl['command']} still pending")
    command_id = uuid.uuid4().hex[:12]
    set_command(CONTROL_FILE, "stop", command_id)
    # 轮询等待 collector flush & finalize（命令清空 + active 变 false）
    session_id = ctrl["recording"].get("session_id")
    for _ in range(60):
        time.sleep(0.5)
        now = read_control(CONTROL_FILE)
        if now.get("command") is None and not now.get("recording", {}).get("active"):
            break
    rec = now.get("recording", {})
    meta = _session_meta(session_id)
    return JSONResponse({
        "status": "stopped",
        "session_id": session_id,
        "sample_count": rec.get("sample_count", 0),
        "size_bytes": rec.get("size_bytes", 0),
        "session": meta,
    })


@app.get("/api/sessions")
def api_sessions(limit: int = 50):
    """列出所有已录制会话（按时间倒序）。"""
    sessions_dir = Path(DATASET_ROOT) / "sessions"
    if not sessions_dir.exists():
        return {"sessions": []}
    sessions = []
    for d in sorted(sessions_dir.iterdir(), key=lambda p: p.name, reverse=True)[:limit]:
        if not d.is_dir():
            continue
        meta = _session_meta(d.name)
        if meta:
            sessions.append(meta)
    return {"sessions": sessions}


def _session_meta(session_id: str) -> Optional[dict]:
    """读取一个会话的 metadata.json 并附加是否已导出标志。"""
    meta_path = Path(DATASET_ROOT) / "sessions" / session_id / "metadata.json"
    if not meta_path.exists():
        return None
    try:
        meta = json.loads(meta_path.read_text("utf-8"))
    except Exception:
        return None
    export_dir = Path(DATASET_ROOT) / "exports" / session_id
    meta["has_export"] = export_dir.exists()
    return meta


@app.get("/api/dataset/export/{session_id}")
def api_dataset_export(session_id: str, train_ratio: float = 0.9):
    """把会话导出为 HuggingFace Dataset 格式（parquet + dataset_info.json）。

    session_id 做白名单校验，防止路径穿越。
    """
    if not session_id.replace("_", "").replace("-", "").isalnum():
        raise HTTPException(400, "invalid session_id")
    try:
        result = build_session_dataset(DATASET_ROOT, session_id, train_ratio=train_ratio)
    except FileNotFoundError:
        raise HTTPException(404, f"session {session_id} not found")
    except Exception as e:
        raise HTTPException(500, f"export failed: {e}")
    return JSONResponse(result)


@app.get("/api/dataset/export/lerobot/{session_id}")
def api_dataset_export_lerobot(session_id: str):
    """把会话导出为 LeRobot v2 格式（episodes parquet + MP4 + meta）。

    若会话有自然语言标注，会写入 tasks.jsonl 并在 parquet 中生成 task_index 列。
    """
    if not session_id.replace("_", "").replace("-", "").isalnum():
        raise HTTPException(400, "invalid session_id")
    try:
        result = build_lerobot_dataset(DATASET_ROOT, session_id)
    except FileNotFoundError:
        raise HTTPException(404, f"session {session_id} not found")
    except Exception as e:
        raise HTTPException(500, f"lerobot export failed: {e}")
    return JSONResponse(result)


# ---------------------------------------------------------------------------
# 定时录制规则管理
# ---------------------------------------------------------------------------

@app.get("/api/schedule")
def api_schedule():
    """列出全部定时规则。"""
    rules = _scheduler.load()
    return {"rules": [r.__dict__ for r in rules]}


@app.post("/api/schedule")
def api_schedule_add(rule: dict):
    """新增一条定时规则（星期 + 开始/停止时间 + 标签）。"""
    day = int(rule.get("day_of_week", 0))
    start_time = str(rule.get("start_time", "00:00"))
    stop_time = str(rule.get("stop_time", "00:30"))
    label = str(rule.get("label", ""))
    enabled = bool(rule.get("enabled", True))
    if not (0 <= day <= 6):
        raise HTTPException(400, "day_of_week must be 0-6")
    new_rule = ScheduleRule(day, start_time, stop_time, label, enabled)
    rules = _scheduler.add(new_rule)
    return {"rules": [r.__dict__ for r in rules]}


@app.delete("/api/schedule/{rule_id}")
def api_schedule_delete(rule_id: str):
    """删除指定定时规则。"""
    ok = _scheduler.delete(rule_id)
    if not ok:
        raise HTTPException(404, "rule not found")
    return {"deleted": True}


@app.post("/api/schedule/{rule_id}/toggle")
def api_schedule_toggle(rule_id: str, enabled: bool = True):
    """启用/禁用指定定时规则。"""
    ok = _scheduler.set_enabled(rule_id, enabled)
    if not ok:
        raise HTTPException(404, "rule not found")
    return {"enabled": enabled}


# ---------------------------------------------------------------------------
# 录制标签
# ---------------------------------------------------------------------------

@app.post("/api/record/label")
def api_record_label(label: str = ""):
    """为当前录制设置标签（录制中调用）。"""
    ctrl = read_control(CONTROL_FILE)
    if ctrl.get("recording", {}).get("active"):
        # 标签写入 control.json，供 collector 在录制元信息中使用
        ctrl["label"] = label
        write_control(CONTROL_FILE, ctrl)
        return {"status": "ok", "label": label}
    raise HTTPException(400, "no active recording")


@app.get("/")
def index():
    """返回前端页面。"""
    return FileResponse(WEB_DIR / "index.html")


# ---------------------------------------------------------------------------
# 帧浏览 + 自然语言标注
# ---------------------------------------------------------------------------

def _session_dir(session_id: str) -> Path:
    """校验并返回 session 目录（白名单校验防路径穿越）。"""
    if not session_id.replace("_", "").replace("-", "").isalnum():
        raise HTTPException(400, "invalid session_id")
    d = Path(DATASET_ROOT) / "sessions" / session_id
    if not d.exists():
        raise HTTPException(404, f"session {session_id} not found")
    return d


def _load_table(session_id: str):
    """把会话的 raw.arrow 读入 pyarrow Table。"""
    import pyarrow.ipc as ipc
    import pyarrow as pa
    raw = _session_dir(session_id) / "raw.arrow"
    if not raw.exists():
        raise HTTPException(404, "session has no raw.arrow")
    with ipc.open_stream(raw) as reader:
        tables = [reader.read_all()]
    return pa.concat_tables(tables)


def _frame_summary(table, i: int) -> dict:
    """提取第 i 帧的紧凑摘要（timestamp、tick、IMU 欧拉角、电机 q、图像名）。"""
    cols = table.column_names
    row = table.slice(i, 1)
    out = {"index": i}
    if "timestamp" in cols:
        out["timestamp"] = float(row.column("timestamp")[0].as_py() or 0)
    if "tick" in cols:
        out["tick"] = int(row.column("tick")[0].as_py() or 0)
    # IMU 欧拉角
    rpy = []
    for c in ("imu_roll_deg", "imu_pitch_deg", "imu_yaw_deg"):
        if c in cols:
            rpy.append(round(float(row.column(c)[0].as_py() or 0), 2))
    if rpy:
        out["imu_rpy"] = rpy
    # 12 个关节角度摘要
    mq = []
    for i in range(12):
        c = f"motor_{i}_q"
        if c in cols:
            mq.append(round(float(row.column(c)[0].as_py() or 0), 3))
    if mq:
        out["motor_q"] = mq
    # 相机图像文件名
    if "image_path" in cols:
        p = row.column("image_path")[0].as_py()
        if p:
            out["image"] = Path(p).name
    return out


def _annotations_path(session_id: str) -> Path:
    """标注文件路径（session 目录下 annotations.json）。"""
    return _session_dir(session_id) / "annotations.json"


def _load_annotations(session_id: str) -> dict:
    """读取会话的标注列表；不存在时返回空结构。"""
    p = _annotations_path(session_id)
    if p.exists():
        try:
            return json.loads(p.read_text("utf-8"))
        except Exception:
            pass
    return {"session_id": session_id, "entries": []}


def _save_annotations(session_id: str, data: dict) -> None:
    """原子写入标注 JSON（先写临时文件再替换）。"""
    p = _annotations_path(session_id)
    tmp = p.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(p)


@app.get("/api/session/{session_id}/frames")
def api_session_frames(session_id: str, start: int = 0, count: int = 50):
    """分页浏览会话的帧摘要（大会话自动抽样，保持响应速度）。"""
    count = max(1, min(count, 500))
    start = max(0, start)
    table = _load_table(session_id)
    total = table.num_rows
    step = max(1, total // 2000)  # 超过 2000 帧时抽样展示
    idxs = list(range(start, min(total, start + count * step), step))
    frames = [_frame_summary(table, i) for i in idxs]
    return {"session_id": session_id, "total": total, "frames": frames}


@app.get("/api/session/{session_id}/frame/{index}")
def api_session_frame(session_id: str, index: int):
    """返回单帧完整摘要。"""
    table = _load_table(session_id)
    if not (0 <= index < table.num_rows):
        raise HTTPException(404, "frame index out of range")
    return JSONResponse(_frame_summary(table, index))


@app.get("/api/session/{session_id}/annotations")
def api_session_annotations(session_id: str):
    """列出会话的全部自然语言标注。"""
    return JSONResponse(_load_annotations(session_id))


@app.post("/api/session/{session_id}/annotations")
def api_session_annotations_add(session_id: str, body: dict):
    """新增一条自然语言标注：指定若干帧 + 描述文本。"""
    frame_indices = body.get("frame_indices", [])
    text = str(body.get("text", "")).strip()
    if not text:
        raise HTTPException(400, "text required")
    if not isinstance(frame_indices, list) or not frame_indices:
        raise HTTPException(400, "frame_indices required")
    frame_indices = [int(x) for x in frame_indices]
    data = _load_annotations(session_id)
    entry = {
        "id": uuid.uuid4().hex[:8],
        "frame_indices": frame_indices,
        "text": text,
        "created_at": time.time(),
    }
    data["entries"].append(entry)
    _save_annotations(session_id, data)
    return JSONResponse(entry)


@app.delete("/api/session/{session_id}/annotations/{entry_id}")
def api_session_annotations_delete(session_id: str, entry_id: str):
    """删除一条标注。"""
    data = _load_annotations(session_id)
    before = len(data["entries"])
    data["entries"] = [e for e in data["entries"] if e["id"] != entry_id]
    if len(data["entries"]) == before:
        raise HTTPException(404, "annotation not found")
    _save_annotations(session_id, data)
    return {"deleted": True}


@app.get("/api/session/{session_id}/image/{filename}")
def api_session_image(session_id: str, filename: str):
    """代理返回会话目录下的相机 JPEG 帧（校验防路径穿越）。"""
    d = _session_dir(session_id)
    img = (d / "images" / filename).resolve()
    if not img.is_file() or not str(img).startswith(str(d)):
        raise HTTPException(404, "image not found")
    return Response(img.read_bytes(), media_type="image/jpeg")


# ---------------------------------------------------------------------------
# 实时相机帧 + 激光点云
# ---------------------------------------------------------------------------

# 机器狗扩展坞 RealSense 深度相机配置（独立采集程序 rs_capture 运行于机器狗）
DOCK_HOST = "192.168.123.18"      # 扩展坞计算板 IP
DOCK_USER = "unitree"
DOCK_PASS = "123"
DOCK_RS_DIR = "/home/unitree/rs_out"   # rs_capture 输出目录
# 本地缓存目录：80服务器定时从机器狗拉取最新深度/彩色帧
DOCK_CACHE_DIR = BASE_DIR / "rs_cache"


def _pull_dock_image(filename: str) -> Optional[bytes]:
    """从机器狗扩展坞拉取一张最新的 RealSense 图像（SFTP）。

    失败返回 None（机器人离线或采集程序未运行）。
    """
    import paramiko
    try:
        cli = paramiko.SSHClient()
        cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        cli.connect(DOCK_HOST, username=DOCK_USER, password=DOCK_PASS, timeout=8)
        sftp = cli.open_sftp()
        with sftp.open(f"{DOCK_RS_DIR}/{filename}", "rb") as f:
            data = f.read()
        sftp.close()
        cli.close()
        return data
    except Exception:
        return None


@app.get("/api/depth/color")
def api_depth_color():
    """返回 RealSense 深度相机的彩色(RGB)图。"""
    data = _pull_dock_image("latest_color.jpg")
    if data is None:
        raise HTTPException(404, "depth camera color unavailable")
    return Response(data, media_type="image/jpeg")


@app.get("/api/depth/image")
def api_depth_image():
    """返回 RealSense 深度相机的伪彩色深度图。"""
    data = _pull_dock_image("latest_depth.jpg")
    if data is None:
        raise HTTPException(404, "depth camera depth unavailable")
    return Response(data, media_type="image/jpeg")


@app.get("/api/depth/status")
def api_depth_status():
    """返回深度相机状态（是否在线 + 最近帧时间）。"""
    color = _pull_dock_image("latest_color.jpg")
    return {
        "camera": "Intel RealSense D435I",
        "online": color is not None,
        "resolution": "640x480",
        "fps": 15,
        "mode": "rs_capture (librealsense)",
    }


@app.get("/api/image/latest")
def api_image_latest():
    """返回最近一帧相机 JPEG（供相机展示区域刷新）。

    优先取当前录制会话的最新帧；否则扫描最近的会话 images/ 目录。
    """
    import glob
    # 1) 优先：当前录制中的最新帧
    st = read_state()
    rec = st.get("recording") or {}
    if rec.get("active") and rec.get("session_id"):
        img_dir = Path(DATASET_ROOT) / "sessions" / rec["session_id"] / "images"
        if img_dir.exists():
            frames = sorted(img_dir.glob("frame_*.jpg"))
            if frames:
                return Response(frames[-1].read_bytes(), media_type="image/jpeg")
    # 2) 回退：最近一个会话的最新帧
    sessions_dir = Path(DATASET_ROOT) / "sessions"
    if sessions_dir.exists():
        latest = None
        for d in sorted(sessions_dir.iterdir(), reverse=True):
            if not d.is_dir():
                continue
            img_dir = d / "images"
            if img_dir.exists():
                frames = sorted(img_dir.glob("frame_*.jpg"))
                if frames:
                    latest = frames[-1]
                    break
        if latest:
            return Response(latest.read_bytes(), media_type="image/jpeg")
    raise HTTPException(404, "no camera image available")


@app.get("/api/lidar/cloud")
def api_lidar_cloud(max_points: int = 3000):
    """采集一次最新激光点云并返回为 xyz 坐标数组。

    数据源为 ROS2 话题 /utlidar/cloud（sensor_msgs/PointCloud2）。
    通过一个短暂的 ROS2 节点订阅获取最新一帧，解析 XYZ 字段并降采样。

    Args:
        max_points: 返回的最大点数（默认 3000，控制浏览器渲染压力）

    Returns:
        {"points": [[x,y,z], ...], "count": N, "hz": 采集频率, "online": bool}
    """
    max_points = max(100, min(max_points, 20000))
    try:
        import rclpy
        from sensor_msgs.msg import PointCloud2
    except ImportError:
        return JSONResponse({"online": False, "reason": "no rclpy on server",
                             "points": [], "count": 0})
    try:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("go2w_lidar_probe")
        latest = {}

        def cb(msg):
            latest["msg"] = msg

        sub = node.create_subscription(PointCloud2, "/utlidar/cloud", cb, 10)
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        import time
        start = time.time()
        while "msg" not in latest and time.time() - start < 4.0:
            executor.spin_once(timeout_sec=0.5)
        node.destroy_node()
        if "msg" not in latest:
            return JSONResponse({"online": False, "reason": "no point cloud in 4s",
                                 "points": [], "count": 0, "hz": 0})
        msg = latest["msg"]
        # 解析 PointCloud2：找到 x/y/z 字段的偏移
        offsets = {}
        for f in msg.fields:
            if f.name in ("x", "y", "z"):
                offsets[f.name] = f.offset
        if len(offsets) != 3:
            return JSONResponse({"online": False, "reason": "cloud lacks xyz fields",
                                 "points": [], "count": 0})
        data = msg.data
        point_step = msg.point_step or 16
        n_points = msg.width
        step = max(1, n_points // max_points)   # 降采样步长
        import struct
        pts = []
        for i in range(0, n_points, step):
            base = i * point_step
            try:
                x = struct.unpack_from("<f", data, base + offsets["x"])[0]
                y = struct.unpack_from("<f", data, base + offsets["y"])[0]
                z = struct.unpack_from("<f", data, base + offsets["z"])[0]
                pts.append([round(x, 3), round(y, 3), round(z, 3)])
            except struct.error:
                break
        return JSONResponse({
            "online": True,
            "points": pts,
            "count": len(pts),
            "total": n_points,
            "hz": round(1.0 / (time.time() - start), 1),
        })
    except Exception as e:
        return JSONResponse({"online": False, "reason": str(e)[:120],
                             "points": [], "count": 0})


@app.get("/favicon.ico")
def favicon():
    """返回站点图标；文件不存在时返回 204 避免 500 报错。"""
    icon = WEB_DIR / "favicon.ico"
    if icon.exists():
        return FileResponse(icon)
    return Response(status_code=204)


# serve static assets (css/js) from web dir
app.mount("/static", StaticFiles(directory=str(WEB_DIR)), name="static")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
