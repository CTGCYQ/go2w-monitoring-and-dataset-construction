"""数据集导出工具（dataset_builder）。

把录制的原始会话（raw.arrow）导出为两种标准的具身智能训练格式：
1. HuggingFace Dataset 格式（build_session_dataset）：列式 parquet + dataset_info.json，
   按 90/10 切分为 train/test，可直接用 `datasets` 库加载；
2. LeRobot v2 格式（build_lerobot_dataset）：episode-per-file 的标准 LeRobot 格式，
   包含 meta/（info.json、episodes.jsonl、tasks.jsonl、stats.json）、
   data/（observation.state + action + task_index 的 parquet）、
   videos/（相机帧合成的 MP4 视频）。若会话有自然语言标注，标注会写入
   tasks.jsonl，并在对应帧的 task_index 列中引用。
"""

from __future__ import annotations

import json
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.ipc as ipc
import pyarrow.parquet as pq

# HuggingFace 格式的字段类型定义
FEATURE_TYPES = {
    "timestamp": "float64",
    "tick": "uint32",
    "imu_quat_w": "float32", "imu_quat_x": "float32", "imu_quat_y": "float32", "imu_quat_z": "float32",
    "imu_gyro_x": "float32", "imu_gyro_y": "float32", "imu_gyro_z": "float32",
    "imu_acc_x": "float32", "imu_acc_y": "float32", "imu_acc_z": "float32",
    "imu_roll_deg": "float32", "imu_pitch_deg": "float32", "imu_yaw_deg": "float32",
    "imu_temp": "float32",
    "power_voltage": "float32", "power_current": "float32",
    "batt_soc": "float32", "batt_current": "float32", "batt_cycle": "float32",
    "batt_bq_ntc": "float32", "batt_mcu_ntc": "float32",
    "sport_mode": "float32", "sport_gait_type": "float32",
    "sport_yaw_speed": "float32", "sport_body_height": "float32", "sport_foot_raise_height": "float32",
}
# 电芯电压 / 电机 / 足底力等字段也是 float32，动态补充
for i in range(10):
    FEATURE_TYPES[f"batt_cell_vol_{i}"] = "float32"
for i in range(12):
    for suffix in ("q", "dq", "tau", "temp"):
        FEATURE_TYPES[f"motor_{i}_{suffix}"] = "float32"
for i in range(4):
    FEATURE_TYPES[f"foot_force_{i}"] = "float32"
    FEATURE_TYPES[f"foot_force_est_{i}"] = "float32"
for i in range(3):
    FEATURE_TYPES[f"sport_pos_{'xyz'[i]}"] = "float32"
    FEATURE_TYPES[f"sport_vel_{'xyz'[i]}"] = "float32"


def read_session_table(session_path: Path) -> pa.Table:
    """读取一个已完成会话的 raw.arrow 为 pyarrow Table。"""
    raw = session_path / "raw.arrow"
    if not raw.exists():
        raise FileNotFoundError(f"no raw.arrow in {session_path}")
    with ipc.open_stream(raw) as reader:
        tables = [reader.read_all()]
    return pa.concat_tables(tables)


def build_session_dataset(
    dataset_root: str,
    session_id: str,
    train_ratio: float = 0.9,
    max_rows_per_file: int = 1_000_000,
) -> dict:
    """把一个会话导出为 HuggingFace Dataset 格式。

    Args:
        dataset_root: 数据集根目录
        session_id: 目标会话 ID
        train_ratio: 训练集占比（剩余为验证集）
        max_rows_per_file: 每个 parquet 文件的最大行数（大文件自动分片）

    Returns:
        导出结果摘要（路径、行数、训练/验证规模等）
    """
    root = Path(dataset_root)
    session_path = root / "sessions" / session_id
    export_path = root / "exports" / session_id
    if not session_path.exists():
        raise FileNotFoundError(f"session {session_id} not found")

    t0 = time.time()
    table = read_session_table(session_path)
    n_rows = table.num_rows
    n_cols = table.num_columns

    # 按行顺序确定性切分训练/验证（不洗牌，保持时序）
    rng = table
    n_train = int(n_rows * train_ratio)
    train_table = rng.slice(0, n_train)
    val_table = rng.slice(n_train, n_rows - n_train)

    if export_path.exists():
        shutil.rmtree(export_path)

    def write_split(split_name: str, t: pa.Table) -> int:
        """把某个 split 写成多个 parquet 分片文件。"""
        out_dir = export_path / "data" / split_name
        out_dir.mkdir(parents=True, exist_ok=True)
        n_files = max(1, (t.num_rows + max_rows_per_file - 1) // max_rows_per_file)
        if t.num_rows == 0:
            return 0
        for i in range(n_files):
            start = i * max_rows_per_file
            chunk = t.slice(start, max_rows_per_file)
            pq.write_table(chunk, out_dir / f"data-{i:05d}-of-{n_files:05d}.parquet")
        return t.num_rows

    n_train_written = write_split("train", train_table)
    n_val_written = write_split("test", val_table)

    # 写 dataset_info.json（字段 schema 与 split 规模）
    features = {k: {"dtype": v} for k, v in FEATURE_TYPES.items()}
    info = {
        "description": f"Go2-W LowState recording session {session_id}",
        "features": features,
        "splits": {
            "train": {"name": "train", "num_examples": n_train_written},
            "test": {"name": "test", "num_examples": n_val_written},
        },
        "config_name": "go2w-lowstate-v1",
        "version": "1.0.0",
    }
    (export_path / "dataset_info.json").write_text(json.dumps(info, indent=2), encoding="utf-8")

    # 附带会话元信息
    meta_src = session_path / "metadata.json"
    if meta_src.exists():
        meta = json.loads(meta_src.read_text("utf-8"))
    else:
        meta = {"session_id": session_id}

    total_bytes = sum(f.stat().st_size for f in (export_path / "data").rglob("*.parquet"))
    return {
        "session_id": session_id,
        "export_path": str(export_path),
        "num_rows": n_rows,
        "num_columns": n_cols,
        "train_rows": n_train_written,
        "val_rows": n_val_written,
        "size_bytes": total_bytes,
        "duration_s": round(time.time() - t0, 2),
        "schema_version": "1.0",
        "session_metadata": meta,
    }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--session", required=True)
    args = parser.parse_args()
    result = build_session_dataset(args.root, args.session)
    print(json.dumps(result, indent=2, ensure_ascii=False))


# ---------------------------------------------------------------------------
# LeRobot v2.x 导出
# ---------------------------------------------------------------------------

# observation.state 的来源列（IMU + 电源 + 电池）
STATE_COLUMNS = (
    "imu_quat_w,imu_quat_x,imu_quat_y,imu_quat_z,"
    "imu_gyro_x,imu_gyro_y,imu_gyro_z,"
    "imu_acc_x,imu_acc_y,imu_acc_z,"
    "imu_roll_deg,imu_pitch_deg,imu_yaw_deg,"
    "power_voltage,power_current,"
    "batt_soc,batt_current,"
).split(",")

# 12 个关节电机的全部字段
MOTOR_COLUMNS = []
for i in range(12):
    for s in ("q", "dq", "tau", "temp"):
        MOTOR_COLUMNS.append(f"motor_{i}_{s}")
FOOT_COLUMNS = [f"foot_force_{i}" for i in range(4)]

# 观测向量 = 状态 + 电机 + 足底力（70 维）
OBSERVATION_COLUMNS = STATE_COLUMNS + MOTOR_COLUMNS + FOOT_COLUMNS
# 动作 = 下一帧与当前帧的电机角度/速度差分（24 维）
ACTION_DELTA_COLUMNS = (
    [f"motor_{i}_q" for i in range(12)] +
    [f"motor_{i}_dq" for i in range(12)]
)

OBS_FIELDS = [
    # 观测向量的语义字段定义（用于 info.json 的 features 描述）
    {"name": "imu_quat", "shape": [4], "dtype": "float32"},
    {"name": "imu_gyro", "shape": [3], "dtype": "float32"},
    {"name": "imu_acc", "shape": [3], "dtype": "float32"},
    {"name": "imu_rpy", "shape": [3], "dtype": "float32"},
    {"name": "power", "shape": [2], "dtype": "float32"},
    {"name": "batt", "shape": [2], "dtype": "float32"},
    {"name": "motor_q", "shape": [12], "dtype": "float32"},
    {"name": "motor_dq", "shape": [12], "dtype": "float32"},
    {"name": "motor_tau", "shape": [12], "dtype": "float32"},
    {"name": "motor_temp", "shape": [12], "dtype": "float32"},
    {"name": "foot_force", "shape": [4], "dtype": "float32"},
]


def _obs_vec(row: dict) -> list[float]:
    """从一行原始状态中提取 70 维观测向量（observation.state）。"""
    vec = []
    for c in OBSERVATION_COLUMNS:
        vec.append(float(row.get(c, 0) or 0))
    return vec


def _action_delta(cur: dict, nxt: dict | None) -> list[float]:
    """计算动作向量：下一帧与当前帧的电机角度/速度差分（24 维）。

    LeRobot 常用增量动作（delta），便于模型学习相对运动而非绝对位姿。
    """
    if nxt is None:
        return [0.0] * len(ACTION_DELTA_COLUMNS)
    act = []
    for c in ACTION_DELTA_COLUMNS:
        delta = float(nxt.get(c, 0) or 0) - float(cur.get(c, 0) or 0)
        act.append(delta)
    return act


def build_lerobot_dataset(
    dataset_root: str,
    session_id: str,
    episode_fps: int = 50,
) -> dict:
    """把一个会话导出为标准 LeRobot v2 格式。

    一个会话 = 一个 episode。若会话有自然语言标注，标注写入 tasks.jsonl，
    并在 parquet 的 task_index 列引用（LeRobot 的标准 task-conditioned 机制）。

    Args:
        dataset_root: 数据集根目录
        session_id: 目标会话 ID
        episode_fps: episode 目标帧率（默认 50，会从 500Hz 降采样）

    Returns:
        导出结果摘要（路径、帧数、观测/动作维度、是否含视频等）
    """
    root = Path(dataset_root)
    session_path = root / "sessions" / session_id
    export_path = root / "exports" / f"{session_id}_lerobot"
    if not session_path.exists():
        raise FileNotFoundError(f"session {session_id} not found")

    t0 = time.time()
    table = read_session_table(session_path)
    rows = table.to_pylist()
    n_rows = len(rows)

    # 从原始 500Hz 降采样到 episode 目标帧率
    orig_hz = 500.0
    skip = max(1, round(orig_hz / episode_fps))
    rows_ds = rows[::skip]

    if export_path.exists():
        shutil.rmtree(export_path)

    # 创建标准 LeRobot 目录结构
    meta_dir = export_path / "meta"
    data_dir = export_path / "data" / "chunk-000"
    video_dir = export_path / "videos" / "chunk-000" / "observation.images.camera"
    for d in (meta_dir, data_dir, video_dir):
        d.mkdir(parents=True, exist_ok=True)

    episode_index = 0
    episode_length = len(rows_ds)

    # 读取自然语言标注，把标注映射为 task（LeRobot 的 task_index 机制）
    annotations = []
    ann_path = session_path / "annotations.json"
    if ann_path.exists():
        try:
            annotations = json.loads(ann_path.read_text("utf-8")).get("entries", [])
        except Exception:
            annotations = []
    tasks = []          # 任务列表：[{"task_index", "task"}]
    task_of_frame = {}  # 帧下标 -> 任务索引
    for a in annotations:
        ti = len(tasks)
        tasks.append({"task_index": ti, "task": a.get("text", "")})
        for f in a.get("frame_indices", []):
            task_of_frame[int(f)] = ti

    # 构建 episode 的 parquet 数据
    obs_list, act_list, ts_list, fi_list, done_list, task_list = [], [], [], [], [], []
    for j, row in enumerate(rows_ds):
        obs = _obs_vec(row)
        nxt = rows_ds[j + 1] if j + 1 < episode_length else None
        act = _action_delta(row, nxt)
        obs_list.append(obs)
        act_list.append(act)
        ts_list.append(float(row.get("timestamp", 0)))
        fi_list.append(j)
        done_list.append(j == episode_length - 1)
        task_list.append(task_of_frame.get(j, -1))   # 无标注的帧为 -1

    # 写 episode parquet（LeRobot 标准列）
    ep_cols = {
        "observation.state": pa.array(obs_list, type=pa.list_(pa.float32())),
        "action": pa.array(act_list, type=pa.list_(pa.float32())),
        "timestamp": pa.array(ts_list, type=pa.float64()),
        "episode_index": pa.array([episode_index] * episode_length, type=pa.int64()),
        "frame_index": pa.array(fi_list, type=pa.int64()),
        "index": pa.array(fi_list, type=pa.int64()),
        "next.done": pa.array(done_list, type=pa.bool_()),
    }
    if tasks:
        ep_cols["task_index"] = pa.array(task_list, type=pa.int64())
    ep_table = pa.table(ep_cols)
    pq.write_table(ep_table, data_dir / "episode_000000.parquet")

    # 观测 / 动作维度
    obs_dim = len(OBSERVATION_COLUMNS)
    act_dim = len(ACTION_DELTA_COLUMNS)

    # info.json：数据集权威 schema
    info = {
        "codebase_version": "v2.0",
        "fps": episode_fps,
        "robot_type": "go2w",
        "total_episodes": 1,
        "total_frames": episode_length,
        "features": {
            "observation.state": {"dtype": "float32", "shape": [obs_dim]},
            "action": {"dtype": "float32", "shape": [act_dim]},
            "timestamp": {"dtype": "float32", "shape": [1]},
        },
    }
    if tasks:
        info["total_tasks"] = len(tasks)
        info["features"]["task_index"] = {"dtype": "int64", "shape": [1]}
    (meta_dir / "info.json").write_text(json.dumps(info, indent=2))

    # episodes.jsonl：episode 级元信息
    ep_task_list = list(dict.fromkeys(ti for ti in task_list if ti >= 0)) if tasks else []
    ep_meta = {"episode_index": 0, "tasks": ep_task_list, "length": episode_length}
    (meta_dir / "episodes.jsonl").write_text(json.dumps(ep_meta) + "\n")

    # tasks.jsonl：自然语言标注任务（无标注则为空）
    if tasks:
        (meta_dir / "tasks.jsonl").write_text(
            "".join(json.dumps(t) + "\n" for t in tasks), encoding="utf-8"
        )
    else:
        (meta_dir / "tasks.jsonl").write_text("")

    # stats.json：观测/动作的归一化统计（mean/std/min/max）
    stats = _compute_stats(obs_list, act_list)
    (meta_dir / "stats.json").write_text(json.dumps(stats, indent=2))

    # 由相机 JPEG 帧序列合成 MP4 视频
    images_dir = session_path / "images"
    video_path = video_dir / "episode_000000.mp4"
    has_video = False
    if images_dir.exists() and list(images_dir.glob("*.jpg")):
        has_video = _images_to_mp4(images_dir, str(video_path), episode_fps)
        if has_video:
            info["features"]["observation.images.camera"] = {
                "dtype": "video", "shape": [3, 480, 640],
            }

    if has_video:
        (meta_dir / "info.json").write_text(json.dumps(info, indent=2))

    total_size = sum(
        f.stat().st_size for f in export_path.rglob("*") if f.is_file()
    )
    return {
        "session_id": session_id,
        "export_path": str(export_path),
        "format": "lerobot-v2",
        "num_episodes": 1,
        "num_frames": episode_length,
        "obs_dim": obs_dim,
        "act_dim": act_dim,
        "num_tasks": len(tasks),
        "has_video": has_video,
        "size_bytes": total_size,
        "duration_s": round(time.time() - t0, 2),
    }


def _compute_stats(obs: list, act: list) -> dict:
    """计算观测/动作的逐维归一化统计（mean/std/min/max），写入 stats.json。

    LeRobot 训练加载器用这些统计做输入归一化。
    """
    obs_arr = np.array(obs, dtype=np.float32)
    act_arr = np.array(act, dtype=np.float32)
    return {
        "observation.state": {
            "mean": obs_arr.mean(axis=0).tolist(),
            "std": obs_arr.std(axis=0).tolist(),
            "min": obs_arr.min(axis=0).tolist(),
            "max": obs_arr.max(axis=0).tolist(),
        },
        "action": {
            "mean": act_arr.mean(axis=0).tolist(),
            "std": act_arr.std(axis=0).tolist(),
            "min": act_arr.min(axis=0).tolist(),
            "max": act_arr.max(axis=0).tolist(),
        },
    }


def _images_to_mp4(images_dir: Path, output: str, fps: int) -> bool:
    """把一组 JPEG 帧序列合成为 MP4 视频（优先 ffmpeg，失败回退 opencv）。

    Args:
        images_dir: 存放 frame_*.jpg 的目录
        output: 输出 MP4 文件路径
        fps: 视频帧率

    Returns:
        是否成功生成 MP4
    """
    # 优先用 ffmpeg（更快、压缩率更高）
    try:
        subprocess.run(
            [
                "ffmpeg", "-y", "-framerate", str(fps),
                "-pattern_type", "glob", "-i",
                str(images_dir / "frame_*.jpg"),
                "-c:v", "libx264", "-preset", "fast", "-crf", "23",
                "-pix_fmt", "yuv420p", output,
            ],
            capture_output=True, timeout=300,
        )
        if Path(output).exists():
            return True
    except Exception:
        pass

    # ffmpeg 不可用时的 opencv 回退
    try:
        import cv2
    except ImportError:
        print("[lerobot] cv2 not available, skip video", flush=True)
        return False

    frames = sorted(images_dir.glob("frame_*.jpg"))
    if not frames:
        return False
    first = cv2.imread(str(frames[0]))
    if first is None:
        return False
    h, w = first.shape[:2]
    writer = cv2.VideoWriter(output, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for f in frames:
        img = cv2.imread(str(f))
        if img is not None:
            writer.write(img)
    writer.release()
    return Path(output).exists()
