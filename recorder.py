"""会话录制引擎（SessionRecorder）。

负责把采集到的 Go2-W 状态帧流式写入 Arrow IPC 文件：
- 录制期间采用「流式写入」策略：每积累 500 帧（约 1 秒）就 flush 一个
  RecordBatch 到磁盘，因此任意长度的录制都不会占满内存；
- 停止录制时把 `_in_progress/` 下的临时文件正式移动到 `sessions/` 目录，
  并生成 metadata.json 描述会话元信息；
- 崩溃恢复：启动时扫描 `_in_progress/` 中残留的半成品文件，自动回收。

数据 schema（Arrow）包含 100 列：timestamp、tick、image_path（相机图像引用）、
IMU 四元数/陀螺仪/加速度/欧拉角/温度、电源电压/电流、电池 BMS、12 个关节电机
（q/dq/tau/temp）、4 足底力、运动模式字段。
"""

from __future__ import annotations

import json    # metadata.json 读写
import shutil  # 文件移动
import threading  # 线程锁，保证回调线程与主循环安全共享
import time    # 时间戳与时长计算
from pathlib import Path

import pyarrow as pa     # Arrow 列式内存格式
import pyarrow.ipc as ipc  # Arrow IPC 流式读写


def make_schema() -> pa.Schema:
    """构造录制文件的 Arrow schema（字段与顺序必须与 flatten_state 一致）。"""
    fields: list[pa.Field] = [
        pa.field("timestamp", pa.float64()),   # 采集时间戳（Unix 秒）
        pa.field("tick", pa.uint32()),         # 机器人内部 tick
        pa.field("image_path", pa.string()),   # 对应相机帧的文件路径（可为空）
    ]
    # IMU 字段
    imu = [
        "imu_quat_w", "imu_quat_x", "imu_quat_y", "imu_quat_z",   # 四元数
        "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",                 # 角速度
        "imu_acc_x", "imu_acc_y", "imu_acc_z",                    # 加速度
        "imu_roll_deg", "imu_pitch_deg", "imu_yaw_deg",           # 欧拉角
        "imu_temp",                                               # IMU 温度
    ]
    # 电源
    power = ["power_voltage", "power_current"]
    # 电池 BMS
    batt = [
        "batt_soc", "batt_current", "batt_cycle",
        "batt_cell_vol_0", "batt_cell_vol_1", "batt_cell_vol_2", "batt_cell_vol_3",
        "batt_cell_vol_4", "batt_cell_vol_5", "batt_cell_vol_6", "batt_cell_vol_7",
        "batt_cell_vol_8", "batt_cell_vol_9",
        "batt_bq_ntc", "batt_mcu_ntc",
    ]
    # 12 个关节电机：q(角度) dq(速度) tau(力矩) temp(温度)
    motors = []
    for i in range(12):
        motors += [f"motor_{i}_q", f"motor_{i}_dq", f"motor_{i}_tau", f"motor_{i}_temp"]
    # 足底力（4 足实际值 + 4 足估计值）
    foot = [
        "foot_force_0", "foot_force_1", "foot_force_2", "foot_force_3",
        "foot_force_est_0", "foot_force_est_1", "foot_force_est_2", "foot_force_est_3",
    ]
    # 运动模式
    sport = [
        "sport_mode", "sport_gait_type",
        "sport_pos_x", "sport_pos_y", "sport_pos_z",
        "sport_vel_x", "sport_vel_y", "sport_vel_z",
        "sport_yaw_speed", "sport_body_height", "sport_foot_raise_height",
    ]
    for name in imu + power + batt + motors + foot:
        fields.append(pa.field(name, pa.float32()))
    for name in sport:
        fields.append(pa.field(name, pa.float32()))
    return pa.schema(fields)


SCHEMA = make_schema()   # 全局复用同一个 schema


def _f(x):
    """安全地转 float，异常值（ctypes/None/bytes/NaN）统一返回 0.0。"""
    try:
        v = float(x)
        return v if v == v else 0.0  # NaN -> 0.0
    except (TypeError, ValueError):
        return 0.0


def flatten_state(st: dict, sport: dict | None, ts: float, image_path: str | None = None) -> list:
    """把 extract_lowstate 返回的字典 + 运动状态压平为一行 schema 顺序的数据。

    Args:
        st: extract_lowstate 的返回（IMU/电机/电池/足底力等）
        sport: 最近一帧运动模式状态（低帧率，前向填充）
        ts: 采集时间戳（Unix 秒）
        image_path: 当前帧对应的相机图像文件路径
    """
    imu = st.get("imu", {})
    quat = imu.get("quaternion") or [0, 0, 0, 0]
    gyro = imu.get("gyroscope") or [0, 0, 0]
    acc = imu.get("accelerometer") or [0, 0, 0]
    rpy = imu.get("rpy_deg") or [0, 0, 0]
    power = st.get("power", {})
    bms = st.get("battery", {})
    cell = bms.get("cell_vol") or []
    motors = st.get("motor", [])

    row = [ts, int(st.get("tick", 0)), image_path or ""]
    # IMU：四元数(4) + 陀螺仪(3) + 加速度(3) + 欧拉角(3) + 温度(1)
    row += [_f(v) for v in quat[:4]]
    row += [_f(v) for v in gyro[:3]]
    row += [_f(v) for v in acc[:3]]
    row += [_f(v) for v in rpy[:3]]
    row += [_f(imu.get("temperature"))]
    # 电源 + 电池
    row += [_f(power.get("voltage")), _f(power.get("current"))]
    row += [_f(bms.get("soc")), _f(bms.get("current")), _f(bms.get("cycle"))]
    row += [_f(cell[i]) if i < len(cell) else 0.0 for i in range(10)]
    row += [_f(bms.get("bq_ntc")), _f(bms.get("mcu_ntc"))]
    # 12 个电机
    for i in range(12):
        m = motors[i] if i < len(motors) else {}
        row += [_f(m.get("q")), _f(m.get("dq")), _f(m.get("tau_est")), _f(m.get("temperature"))]
    # 足底力
    ff = st.get("foot_force") or []
    ffe = st.get("foot_force_est") or []
    row += [_f(ff[i]) if i < len(ff) else 0.0 for i in range(4)]
    row += [_f(ffe[i]) if i < len(ffe) else 0.0 for i in range(4)]
    # 运动模式（前向填充最近值）
    sp = sport or {}
    pos = sp.get("position") or [0, 0, 0]
    vel = sp.get("velocity") or [0, 0, 0]
    row += [_f(sp.get("mode")), _f(sp.get("gait_type"))]
    row += [_f(v) for v in pos[:3]]
    row += [_f(v) for v in vel[:3]]
    row += [_f(sp.get("yaw_speed")), _f(sp.get("body_height")), _f(sp.get("foot_raise_height"))]
    return row


class SessionRecorder:
    """一次录制会话的流式记录器。"""

    def __init__(self, dataset_root: str):
        self.root = Path(dataset_root)
        self.in_progress_dir = self.root / "_in_progress"  # 录制中的临时目录
        self.sessions_dir = self.root / "sessions"          # 已完成会话目录
        self.exports_dir = self.root / "exports"            # 导出数据目录
        for d in (self.root, self.in_progress_dir, self.sessions_dir, self.exports_dir):
            d.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()     # 保护所有内部状态
        self._active = False              # 是否正在录制
        self._session_id = None           # 当前会话 ID
        self._start_ts = None             # 会话开始时间
        self._sample_count = 0            # 已录制帧数
        self._label = ""                  # 会话标签
        self._writer: ipc.RecordBatchStreamWriter | None = None  # Arrow 流写入器
        self._buf = []                    # 行缓冲（攒够一批再 flush）
        self._flush_interval = 500        # 每 500 帧 flush 一次（约 1 秒 @500Hz）
        self._last_sport: dict | None = None  # 最近一帧运动状态
        self._error = None                # 录制错误信息

    # ---- 状态查询 ----
    def is_active(self) -> bool:
        """当前是否处于录制中。"""
        return self._active

    def status(self) -> dict:
        """返回录制状态摘要（供 state.json / control.json 展示）。"""
        with self._lock:
            return {
                "active": self._active,
                "session_id": self._session_id,
                "start_ts": self._start_ts,
                "sample_count": self._sample_count,
                "size_bytes": self._size_bytes(),
                "error": self._error,
            }

    def _size_bytes(self) -> int:
        """当前正在写入的 raw.arrow 文件大小。"""
        if not self._session_id:
            return 0
        f = self.in_progress_dir / self._session_id / "raw.arrow"
        return f.stat().st_size if f.exists() else 0

    # ---- 控制 ----
    def set_sport(self, sport: dict) -> None:
        """缓存最新运动状态（供低帧率话题前向填充）。"""
        with self._lock:
            self._last_sport = sport

    def start(self, session_id: str, label: str = "") -> None:
        """开始一个新会话：在 _in_progress/ 下创建目录并打开 Arrow 流。"""
        with self._lock:
            if self._active:
                return
            self._active = True
            self._session_id = session_id
            self._start_ts = time.time()
            self._sample_count = 0
            self._error = None
            self._buf = []
            self._label = label
            session_dir = self.in_progress_dir / session_id
            session_dir.mkdir(parents=True, exist_ok=True)
            self._writer = ipc.new_stream(session_dir / "raw.arrow", SCHEMA)

    def ingest(self, st: dict, ts: float, image_path: str | None = None) -> None:
        """写入一帧状态。录制期间由采集回调逐帧调用。"""
        with self._lock:
            if not self._active:
                return
            row = flatten_state(st, self._last_sport, ts, image_path)
            self._buf.append(row)
            self._sample_count += 1
            # 攒够一批就落盘
            if len(self._buf) >= self._flush_interval:
                self._flush_locked()

    def set_label(self, label: str) -> None:
        """更新当前会话标签。"""
        with self._lock:
            self._label = label

    def stop(self) -> dict:
        """停止录制：flush 剩余数据、关闭流、移动到 sessions/ 并写元信息。"""
        with self._lock:
            if not self._active:
                return self.status()
            try:
                self._flush_locked()
            finally:
                self._active = False
            n = self._sample_count
            session_id = self._session_id
            start = self._start_ts
            size = self._size_bytes()
            writer = self._writer
            self._writer = None
            # 关闭 Arrow 流，确保文件完整
            if writer is not None:
                writer.close()
            # 把 _in_progress/<id> 下的内容移动到 sessions/<id>
            src = self.in_progress_dir / session_id
            dst = self.sessions_dir / session_id
            dst.mkdir(parents=True, exist_ok=True)
            # 移动 raw.arrow
            raw_src = src / "raw.arrow"
            if raw_src.exists():
                if (dst / "raw.arrow").exists():
                    (dst / "raw.arrow").unlink()
                shutil.move(str(raw_src), str(dst / "raw.arrow"))
            # 移动相机 images 目录（若存在）
            img_src = src / "images"
            if img_src.exists():
                img_dst = dst / "images"
                if img_dst.exists():
                    shutil.rmtree(img_dst, ignore_errors=True)
                shutil.move(str(img_src), str(img_dst))
            shutil.rmtree(src, ignore_errors=True)

            # 写会话元信息
            duration = time.time() - start if start else 0
            meta = {
                "session_id": session_id,
                "start_ts": start,
                "duration_s": round(duration, 3),
                "sample_count": n,
                "size_bytes": size,
                "effective_hz": round(n / duration, 2) if duration > 0 else 0.0,
                "label": self._label,
                "crashed": False,
                "schema_version": "1.0",
            }
            (dst / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
            return {"active": False, "session_id": session_id, **meta}

    def _flush_locked(self) -> None:
        """把行缓冲转成 Arrow RecordBatch 写入磁盘（须持有锁调用）。"""
        if not self._buf or self._writer is None:
            return
        batch = pa.RecordBatch.from_arrays(
            [pa.array(cols) for cols in zip(*self._buf)], schema=SCHEMA
        )
        self._writer.write_batch(batch)
        self._buf = []

    def recover_crashed(self) -> list[str]:
        """回收采集器崩溃后残留在 _in_progress/ 的半成品会话。

        Arrow IPC 流是自描述的，重读可以取到最后一个完整 batch。
        恢复的会话标记 crashed=True。
        """
        recovered = []
        for d in self.in_progress_dir.iterdir():
            if d.is_dir() and (d / "raw.arrow").exists():
                session_id = d.name
                try:
                    src = d / "raw.arrow"
                    with ipc.open_stream(src) as reader:
                        n = sum(1 for _ in reader)   # 统计完整 batch 数
                    dst = self.sessions_dir / session_id
                    dst.mkdir(parents=True, exist_ok=True)
                    shutil.move(str(src), str(dst / "raw.arrow"))
                    shutil.rmtree(d, ignore_errors=True)
                    meta = {
                        "session_id": session_id,
                        "start_ts": src.stat().st_mtime,
                        "duration_s": 0,
                        "sample_count": n * self._flush_interval,
                        "size_bytes": src.stat().st_size,
                        "effective_hz": 0.0,
                        "crashed": True,
                        "schema_version": "1.0",
                    }
                    (dst / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
                    recovered.append(session_id)
                except Exception as e:
                    print(f"[recorder] recover failed {session_id}: {e}", flush=True)
        return recovered
