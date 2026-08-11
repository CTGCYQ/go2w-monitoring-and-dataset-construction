"""控制文件协议（control.json）—— collector 与 server 之间的命令通道。

Web 端通过 server 触发录制，server 把命令写入 control.json；collector 每秒轮询
该文件并按命令执行（开始/停止录制），执行完成后通过 acknowledge 回写状态。

为什么不用额外 HTTP 接口？
- collector 是单进程 DDS 订阅者，不想给它再加一个 HTTP 监听端口
- 文件通道简单可靠，且与 state.json 的既有通信模式一致

控制文件结构：
{
  "command": "start" | "stop" | null,   # 待处理命令（null 表示无命令）
  "command_id": "abc123",               # 命令唯一 ID
  "command_ts": 1754880000.123,         # 命令下发时间
  "label": "walking",                   # 本次录制的标签
  "recording": {                        # 录制状态（collector 回写）
      "active": false, "session_id": null, "start_ts": null,
      "sample_count": 0, "size_bytes": 0, "error": null
  }
}
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

# 控制文件的默认结构（recording 字段由 collector 实时更新）
DEFAULT_CONTROL = {
    "command": None,          # "start" | "stop" | None
    "command_id": None,
    "command_ts": None,
    "label": "",              # 下一次录制的可选标签
    "recording": {
        "active": False,
        "session_id": None,
        "start_ts": None,
        "sample_count": 0,
        "size_bytes": 0,
        "error": None,
    },
}


def read_control(path: Path) -> dict:
    """读取控制文件；不存在或损坏时返回默认结构。"""
    if not path.exists():
        return json.loads(json.dumps(DEFAULT_CONTROL))
    try:
        return json.loads(path.read_text("utf-8"))
    except Exception:
        return json.loads(json.dumps(DEFAULT_CONTROL))


def write_control(path: Path, ctrl: dict) -> None:
    """原子写入控制文件（先写临时文件再 replace，避免半写状态）。"""
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(ctrl, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def set_command(path: Path, command: str, command_id: str, label: str = "") -> None:
    """下发给 collector 一条命令（由 server 调用）。"""
    ctrl = read_control(path)
    ctrl["command"] = command
    ctrl["command_id"] = command_id
    ctrl["command_ts"] = time.time()
    ctrl["label"] = label
    write_control(path, ctrl)


def acknowledge(path: Path, recording_state: dict) -> None:
    """collector 确认命令已处理：清空命令，回写录制状态。"""
    ctrl = read_control(path)
    ctrl["command"] = None
    ctrl["command_id"] = None
    ctrl["command_ts"] = None
    ctrl["label"] = ""
    ctrl["recording"] = recording_state
    write_control(path, ctrl)
