"""Go2-W 机器人实时状态采集器（Collector）。

该模块是整个监控系统的数据入口：
1. 通过 unitree_sdk2py 的 DDS 通道订阅 Go2-W 机器人的 `rt/lowstate`（低频状态，约 500Hz）
   和 `rt/lf/sportmodestate`（运动模式状态，约 10Hz）；
2. 将采集到的状态解析为结构化字典（IMU、电机、电池、足底力等）；
3. 实时写入 InfluxDB 用于历史曲线展示；
4. 将最新一帧快照写入 state.json 供 Web 前端轮询；
5. 在"录制模式"下，把每一帧原始状态连同相机图像路径交给 SessionRecorder 落盘；
6. 轮询 control.json 接收 Web 端下发的开始/停止录制命令；
7. 根据 schedule.json 中的定时规则实现无人值守自动采集。

运行方式（在 Go2-W 服务器上，tmux 中常驻）：
    python collector.py --interface enp0s31f6 --database go2w_monitor --camera
"""

from __future__ import annotations

import argparse   # 命令行参数解析
import json       # JSON 读写（state.json / control.json / schedule.json）
import math       # 数学函数（四元数转欧拉角）
import os
import signal     # 捕获 SIGTERM/SIGINT 实现优雅退出
import threading  # 线程锁，保护共享状态 LATEST
import time       # 时间戳与频率计算
import uuid       # 生成录制 session 的唯一 ID
from pathlib import Path

try:
    import requests  # 向 InfluxDB 写入 HTTP 数据
except ImportError:
    requests = None

# 本地模块
from control_schema import acknowledge, read_control, set_command  # 控制文件协议
from recorder import SessionRecorder                                # 录制引擎
from scheduler import ScheduleManager                               # 定时调度
from video_worker import VideoWorker                                # 相机采集线程

# 多模态数据源（录制时同步到会话目录）
PC_CACHE = Path(__file__).resolve().parent / "pc_cache.json"   # 激光点云缓存（pc_worker 写入）
DOCK_RS_DIR = "/home/unitree/rs_out"                            # 深度相机输出目录（机器狗扩展坞）


def _snapshot_pointcloud(session_dir: Path) -> Optional[Path]:
    """把最新点云缓存复制到会话目录（返回保存的文件路径）。"""
    import shutil
    try:
        if PC_CACHE.exists():
            cloud_dir = session_dir / "pointclouds"
            cloud_dir.mkdir(parents=True, exist_ok=True)
            import time as _t
            dst = cloud_dir / f"cloud_{int(_t.time())}.json"
            shutil.copy(PC_CACHE, dst)
            return dst
    except Exception as e:
        print(f"[snapshot] pointcloud error: {e}", flush=True)
    return None


def _snapshot_depth(session_dir: Path) -> None:
    """把深度相机最新帧复制到会话目录（机器狗扩展坞上的 rs_capture 输出）。"""
    import shutil
    try:
        import paramiko as _paramiko
        cli = _paramiko.SSHClient()
        cli.set_missing_host_key_policy(_paramiko.AutoAddPolicy())
        cli.connect("192.168.123.18", username="unitree", password="123", timeout=5)
        sftp = cli.open_sftp()
        depth_dir = session_dir / "depth"
        color_dir = session_dir / "depth_color"
        depth_dir.mkdir(parents=True, exist_ok=True)
        color_dir.mkdir(parents=True, exist_ok=True)
        import time as _t
        ts = int(_t.time())
        for remote, local_dir in (("latest_depth.jpg", depth_dir), ("latest_color.jpg", color_dir)):
            try:
                sftp.get(f"{DOCK_RS_DIR}/{remote}", str(local_dir / f"{ts}_{remote}"))
            except Exception:
                pass
        sftp.close()
        cli.close()
    except Exception as e:
        print(f"[snapshot] depth error: {e}", flush=True)

# ---------------------------------------------------------------------------
# 全局路径与状态
# ---------------------------------------------------------------------------
STATE_PATH = Path(__file__).resolve().parent / "state.json"      # 最新状态快照
CONTROL_PATH = Path(__file__).resolve().parent / "control.json"  # 控制命令文件
SCHEDULE_PATH = Path(__file__).resolve().parent / "schedule.json"  # 定时规则文件
LOCK = threading.Lock()          # 保护 LATEST 字典的线程锁
_recorder: SessionRecorder | None = None   # 录制引擎（惰性初始化）
_scheduler: ScheduleManager | None = None  # 定时调度器
_video: VideoWorker | None = None          # 相机线程
_auto_rule_id: str | None = None           # 当前由哪条定时规则发起的自动录制

# 最新一帧状态的缓存字典，save() 每秒序列化到 state.json
LATEST: dict[str, object] = {
    "online": False,        # 机器人是否在线（3 秒无数据视为离线）
    "last_seen_s": None,    # 最近一次收到数据的时间戳（秒）
    "hz": 0.0,              # 实时采集频率
    "power": {},            # 电源电压/电流
    "imu": {},              # IMU 姿态数据
    "motor": [],            # 12 个关节电机状态
    "foot_force": [],       # 足底力
    "battery": {},          # 电池 BMS 状态
    "tick": 0,              # 机器人内部 tick 计数
    "sport_mode": {},       # 运动模式状态
}


def q2a(x: float) -> float:
    """四元数 (w, x, y, z) 转欧拉角 (roll, pitch, yaw)，单位为度。

    机器人姿态通常以四元数表示，但直观展示用欧拉角更方便。
    转换公式来自四元数与欧拉角的经典换算关系。
    """
    w, x, y, z = x[0], x[1], x[2], x[3]
    norm = math.sqrt(w * w + x * x + y * y + z * z) or 1.0   # 归一化，防止非单位四元数
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _to_list(x):
    """将 ctypes 数组 / bytes / 可迭代对象统一转为 float 列表。

    unitree 的 DDS 消息中，传感器数据可能是 ctypes 数组或 bytes，
    转成 Python 原生 list 便于 JSON 序列化和下游处理。
    """
    if hasattr(x, "__len__") and not isinstance(x, (str, bytes)):
        return [float(v) for v in x]
    if isinstance(x, bytes):
        return [float(v) for v in x]
    return x


def extract_lowstate(msg) -> dict[str, object]:
    """从 DDS 的 LowState_ 消息中提取全部传感器状态为字典。

    返回结构包含：tick、imu（四元数/陀螺仪/加速度/欧拉角/温度）、
    power（电压/电流）、battery（电池 BMS）、motor（12 关节）、foot_force。
    """
    out: dict[str, object] = {"tick": msg.tick}

    # IMU 惯性测量单元
    imu = msg.imu_state
    quat = _to_list(imu.quaternion)
    out["imu"] = {
        "quaternion": quat,                              # 原始四元数
        "gyroscope": _to_list(imu.gyroscope),            # 角速度 (rad/s)
        "accelerometer": _to_list(imu.accelerometer),    # 线加速度 (m/s²)
        "rpy_deg": q2a(quat),                            # 欧拉角 (度)
        "temperature": imu.temperature,                  # IMU 芯片温度
    }

    # 电源与电池
    bms = msg.bms_state
    out["power"] = {"voltage": msg.power_v, "current": msg.power_a}
    out["battery"] = {
        "soc": bms.soc,               # 电池电量百分比
        "current": bms.current,       # 电池电流
        "cycle": bms.cycle,           # 充放电循环次数
        "cell_vol": _to_list(bms.cell_vol),  # 各电芯电压
        "bq_ntc": bms.bq_ntc,         # 电芯温度
        "mcu_ntc": bms.mcu_ntc,       # MCU 温度
    }

    # 12 个关节电机
    motors = []
    for i, m in enumerate(msg.motor_state):
        motors.append(
            {
                "index": i,            # 关节编号 0-11
                "q": m.q,              # 关节角度 (rad)
                "dq": m.dq,            # 关节角速度 (rad/s)
                "tau_est": m.tau_est,  # 估计力矩 (N·m)
                "temperature": m.temperature,  # 电机温度
            }
        )
    out["motor"] = motors

    # 足底力（4 足接触力 + 估计力）
    out["foot_force"] = _to_list(msg.foot_force)
    out["foot_force_est"] = _to_list(msg.foot_force_est)

    return out


def extract_sport(msg) -> dict[str, object]:
    """从 SportModeState_ 消息中提取运动模式状态。

    包含模式编号、步态类型、位置/速度（机身坐标系）、偏航角速度、机身高度等。
    """
    out = {
        "mode": msg.mode,                     # 运动模式枚举
        "gait_type": msg.gait_type,           # 步态类型
        "position": _to_list(msg.position),   # 机身位置 [x, y, z]
        "velocity": _to_list(msg.velocity),   # 机身线速度 [vx, vy, vz]
        "yaw_speed": msg.yaw_speed,           # 偏航角速度
        "body_height": msg.body_height,       # 机身高度
        "foot_raise_height": msg.foot_raise_height,  # 足底抬起高度
    }
    return out


class InfluxWriter:
    """InfluxDB 时序数据库写入器（InfluxDB 1.x Line Protocol）。"""

    def __init__(self, url: str, database: str):
        self.url = url.rstrip("/")
        self.database = database

    def write(self, points: list[str]) -> None:
        """批量写入一组 Line Protocol 数据点。"""
        if requests is None:
            return
        data = "\n".join(points)
        try:
            r = requests.post(
                f"{self.url}/write?db={self.database}&precision=ms",
                data=data,
                timeout=2,
            )
            if r.status_code not in (204, 200):
                print(f"[influx] write failed: {r.status_code} {r.text[:120]}", flush=True)
        except Exception as e:
            print(f"[influx] write error: {e}", flush=True)


def build_line(meas: str, tags: dict[str, str], fields: dict[str, float]) -> str:
    """构造一行 InfluxDB Line Protocol 字符串。

    格式：<measurement>,<tag_key>=<tag_val> <field_key>=<field_val>
    """
    tag_str = ",".join(f"{k}={v}" for k, v in sorted(tags.items()))
    field_str = ",".join(f"{k}={v}" for k, v in fields.items())
    return f"{meas},{tag_str} {field_str}"


def run(interface: str, influx_url: str, database: str, interval: float,
        dataset_root: str, enable_camera: bool = False) -> None:
    """采集器主循环（阻塞运行）。

    Args:
        interface: 机器人专网网卡（如 enp0s31f6）
        influx_url: InfluxDB HTTP 地址
        database: InfluxDB 数据库名
        interval: InfluxDB 写入降采样间隔（秒）
        dataset_root: 录制数据集的根目录
        enable_camera: 是否启用前置相机采集
    """
    # 延迟导入 unitree SDK（仅在服务器环境可用）
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

    global _recorder, _scheduler, _video
    _recorder = SessionRecorder(dataset_root)   # 初始化录制引擎
    _recorder.recover_crashed()                 # 恢复上次异常中断的录制
    _scheduler = ScheduleManager(str(SCHEDULE_PATH))

    # 先初始化 DDS ChannelFactory（单例），再启动相机线程。
    # 相机线程会复用这个工厂，避免重复初始化导致的冲突。
    ChannelFactoryInitialize(0, interface)
    if enable_camera:
        _video = VideoWorker(interface, dataset_root, fps=15)
        _video.start()
        print("[collector] camera enabled", flush=True)

    writer = InfluxWriter(influx_url, database)
    count = [0]            # 收到的消息计数（用于降采样）
    last_ts = [time.time()]  # 上一帧时间
    hz = [0.0]             # 实时频率（滑动窗口平均）
    hz_count = [0]         # 滑动窗口内的帧计数
    hz_start = [time.time()]  # 滑动窗口起始时间
    _last_sched_check = 0.0  # 上次检查定时规则的时间
    _last_snap = 0.0        # 上次多模态快照的时间

    def on_lowstate(msg, _name=None) -> None:
        """低频状态回调（约 500Hz）。"""
        now = time.time()
        count[0] += 1
        dt = now - last_ts[0]
        # 计算实时频率（滑动窗口：每秒统计一次帧数，更稳定）
        if dt > 0.5:
            hz[0] = 0.0
            last_ts[0] = now
            hz_count[0] = 0
            hz_start[0] = now
        else:
            last_ts[0] = now
            hz_count[0] += 1
            if now - hz_start[0] >= 1.0:
                hz[0] = hz_count[0] / max(now - hz_start[0], 1e-6)
                hz_count[0] = 0
                hz_start[0] = now

        try:
            st = extract_lowstate(msg)
        except Exception as e:
            print(f"[parse] {e}", flush=True)
            return

        # 更新最新快照（供 state.json）
        with LOCK:
            LATEST.update(st)
            LATEST["online"] = True
            LATEST["last_seen_s"] = now
            LATEST["hz"] = hz[0]

        # 录制模式下，把每一帧状态 + 当前相机图像路径交给录制引擎
        if _recorder.is_active():
            img_path = _video.latest_image_path() if _video else None
            _recorder.ingest(st, now, img_path)

        # InfluxDB 写入（按 interval 降采样，避免 500Hz 高频写入）
        if count[0] % max(1, int(interval * 10)) == 0:
            ts_ms = int(now * 1000)
            pts = []
            # 电源
            pts.append(build_line("go2w_power", {"robot": "go2w"},
                                  {"voltage": msg.power_v, "current": msg.power_a}))
            # IMU 陀螺仪 + 加速度
            pts.append(build_line("go2w_imu", {"robot": "go2w"},
                                  {"gyro_x": msg.imu_state.gyroscope[0],
                                   "gyro_y": msg.imu_state.gyroscope[1],
                                   "gyro_z": msg.imu_state.gyroscope[2],
                                   "acc_x": msg.imu_state.accelerometer[0],
                                   "acc_y": msg.imu_state.accelerometer[1],
                                   "acc_z": msg.imu_state.accelerometer[2]}))
            # IMU 欧拉角
            pts.append(build_line("go2w_imu_rpy", {"robot": "go2w"},
                                  {"roll": st["imu"]["rpy_deg"][0],
                                   "pitch": st["imu"]["rpy_deg"][1],
                                   "yaw": st["imu"]["rpy_deg"][2]}))
            # 电池
            bms = msg.bms_state
            pts.append(build_line("go2w_battery", {"robot": "go2w"},
                                  {"soc": bms.soc, "current": bms.current,
                                   "cell_vol_max": max(bms.cell_vol) if bms.cell_vol else 0.0,
                                   "cell_vol_min": min(bms.cell_vol) if bms.cell_vol else 0.0}))
            # 12 个电机
            for i, m in enumerate(msg.motor_state):
                pts.append(build_line("go2w_motor", {"robot": "go2w", "joint": str(i)},
                                      {"q": m.q, "dq": m.dq, "tau": m.tau_est}))
            # 足底力
            pts.append(build_line("go2w_foot_force", {"robot": "go2w"},
                                  {f"f{i}": v for i, v in enumerate(msg.foot_force)}))
            writer.write(pts)

    def on_sport(msg, _name=None) -> None:
        """运动模式状态回调（约 10Hz）。"""
        try:
            sport = extract_sport(msg)
            with LOCK:
                LATEST["sport_mode"] = sport
            # 把最新运动状态传给录制引擎（用于前向填充，低帧率话题）
            _recorder.set_sport(sport)
        except Exception as e:
            print(f"[sport] {e}", flush=True)

    # 建立 DDS 订阅
    subs = []
    low = ChannelSubscriber("rt/lowstate", LowState_)
    low.Init(on_lowstate, 10)
    subs.append(low)
    sport = ChannelSubscriber("rt/lf/sportmodestate", SportModeState_)
    sport.Init(on_sport, 10)
    subs.append(sport)

    def save() -> None:
        """把最新快照 + 录制状态序列化写入 state.json（原子写）。"""
        def _jsonable(obj):
            """递归把不可 JSON 序列化的类型（bytes/ndarray/nan）转为可序列化。"""
            if isinstance(obj, bytes):
                return list(obj)
            if isinstance(obj, bytearray):
                return list(obj)
            if hasattr(obj, "tolist"):
                return obj.tolist()
            if isinstance(obj, dict):
                return {k: _jsonable(v) for k, v in obj.items()}
            if isinstance(obj, (list, tuple)):
                return [_jsonable(v) for v in obj]
            if isinstance(obj, float):
                if math.isnan(obj) or math.isinf(obj):
                    return None
                return obj
            return obj

        with LOCK:
            # 附加录制状态，供前端展示
            rec_status = _recorder.status()
            LATEST["recording"] = {
                "active": rec_status["active"],
                "session_id": rec_status["session_id"],
                "start_ts": rec_status["start_ts"],
                "sample_count": rec_status["sample_count"],
                "size_bytes": rec_status["size_bytes"],
            }
            snapshot = json.dumps(_jsonable(LATEST), ensure_ascii=False)
        STATE_PATH.write_text(snapshot, encoding="utf-8")

    # 优雅退出：收到信号后跳出主循环并停掉录制
    stop = threading.Event()

    def on_signal(_sig, _frame) -> None:
        stop.set()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    print(f"[collector] started on {interface}, writing to {influx_url} db={database}", flush=True)
    while not stop.is_set():
        # ---- 处理 Web 端下发的录制控制命令 ----
        ctrl = read_control(CONTROL_PATH)
        if ctrl.get("command") == "start":
            # 开始录制：生成 session ID，启动录制引擎 + 相机
            session_id = uuid.uuid4().hex[:12]
            label = ctrl.get("label", "") or ""
            print(f"[recorder] START session={session_id} label={label!r}", flush=True)
            _recorder.start(session_id, label)
            if _video:
                _video.set_session(session_id)
            acknowledge(CONTROL_PATH, _recorder.status())   # 确认命令已处理
        elif ctrl.get("command") == "stop":
            # 停止录制：先停相机写图（保证 images 目录稳定），再落盘
            print("[recorder] STOP", flush=True)
            if _video:
                _video.set_session(None)
            result = _recorder.stop()
            print(f"[recorder] saved {result.get('sample_count', 0)} samples "
                  f"-> {result.get('session_id')}", flush=True)
            acknowledge(CONTROL_PATH, _recorder.status())

        # ---- 自动定时录制检查（每秒一次） ----
        if time.time() - _last_sched_check >= 1.0:
            _last_sched_check = time.time()
            rule = _scheduler.active_rule()          # 当前是否有命中规则的时段
            now_active = _recorder.is_active()
            if rule and not now_active and LATEST.get("online"):
                # 命中规则且未在录制 → 自动开始
                session_id = uuid.uuid4().hex[:12]
                print(f"[scheduler] auto START rule={rule.rule_id} label={rule.label!r}", flush=True)
                _recorder.start(session_id, rule.label)
                if _video:
                    _video.set_session(session_id)
                global _auto_rule_id
                _auto_rule_id = rule.rule_id
                ctrl2 = read_control(CONTROL_PATH)
                ctrl2["recording"] = _recorder.status()
                from control_schema import write_control
                write_control(CONTROL_PATH, ctrl2)
            elif now_active and _auto_rule_id and not rule:
                # 自动开始的录制，规则时段已过 → 自动停止
                print("[scheduler] auto STOP (rule expired)", flush=True)
                if _video:
                    _video.set_session(None)
                result = _recorder.stop()
                print(f"[recorder] saved {result.get('sample_count', 0)} samples "
                      f"-> {result.get('session_id')}", flush=True)
                _auto_rule_id = None
                ctrl2 = read_control(CONTROL_PATH)
                ctrl2["recording"] = _recorder.status()
                from control_schema import write_control
                write_control(CONTROL_PATH, ctrl2)

        save()
        # 超过 3 秒没有数据 → 判定机器人离线
        with LOCK:
            if LATEST["last_seen_s"] and time.time() - LATEST["last_seen_s"] > 3.0:
                LATEST["online"] = False

        # ---- 录制期间多模态快照（点云 + 深度图，每 3 秒） ----
        if _recorder.is_active() and time.time() - _last_snap >= 3.0:
            _last_snap = time.time()
            st = _recorder.status()
            sid = st.get("session_id")
            if sid:
                session_dir = Path(dataset_root) / "sessions" / sid
                _snapshot_pointcloud(session_dir)
                _snapshot_depth(session_dir)

        time.sleep(1)

    # 退出前若仍在录制，保存数据
    if _recorder.is_active():
        _recorder.stop()
    print("[collector] stopped", flush=True)


def main() -> None:
    """命令行入口：解析参数并启动采集器。"""
    parser = argparse.ArgumentParser(description="Go2-W state collector -> InfluxDB")
    parser.add_argument("--interface", default="enp0s31f6", help="机器人专网网卡")
    parser.add_argument("--influx-url", default="http://127.0.0.1:8091", help="InfluxDB 地址")
    parser.add_argument("--database", default="go2w_monitor", help="InfluxDB 数据库名")
    parser.add_argument("--interval", type=float, default=1.0, help="InfluxDB 写入降采样间隔")
    parser.add_argument("--dataset-root",
                        default="/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-dataset",
                        help="录制数据集根目录")
    parser.add_argument("--camera", action="store_true", help="enable front camera recording")
    args = parser.parse_args()
    run(args.interface, args.influx_url, args.database, args.interval,
        args.dataset_root, enable_camera=args.camera)


if __name__ == "__main__":
    main()
