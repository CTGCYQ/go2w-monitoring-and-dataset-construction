"""定时录制调度（Schedule）。

规则存储在 schedule.json（与 control.json 同目录）。collector 每秒轮询一次
调度管理器，当规则的时间窗口命中当前星期 + 时钟时，自动触发开始/停止录制，
从而实现无人值守的持续数据采集。

一条规则示例：
    {"day_of_week": 0, "start_time": "09:00", "stop_time": "09:30",
     "label": "morning_walk", "enabled": true, "rule_id": "751091e5"}
其中 day_of_week：0=周一 ... 6=周日；时间格式 "HH:MM"；支持跨天时段（如 23:30→00:30）。
"""

from __future__ import annotations

import json
import os
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

DEFAULT_RULES: dict[str, object] = {"rules": []}


def _now() -> datetime:
    """获取当前本地时间。"""
    return datetime.now()


@dataclass
class ScheduleRule:
    """一条定时录制规则。"""

    day_of_week: int            # 0=周一 ... 6=周日
    start_time: str             # 开始时间 "HH:MM"
    stop_time: str              # 停止时间 "HH:MM"
    label: str = ""             # 录制标签（写入 session 元信息）
    enabled: bool = True        # 是否启用
    rule_id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])  # 唯一 ID

    def active_now(self, now: datetime | None = None) -> bool:
        """判断当前时刻是否落在这条规则的时间窗口内（支持跨天时段）。"""
        if not self.enabled:
            return False
        now = now or _now()
        if now.weekday() != self.day_of_week:
            return False
        cur = now.hour * 60 + now.minute          # 当前时刻（分钟）
        start_min = self._to_min(self.start_time)
        stop_min = self._to_min(self.stop_time)
        if start_min == stop_min:
            return False
        if start_min < stop_min:
            # 常规时段：开始 <= 当前 < 停止
            return start_min <= cur < stop_min
        # 跨天时段（如 23:30 → 00:30）：当前 >= 开始 或 当前 < 停止
        return cur >= start_min or cur < stop_min

    def _to_min(self, hm: str) -> int:
        """把 "HH:MM" 字符串转换为当天分钟数。"""
        try:
            h, m = hm.split(":")
            return int(h) * 60 + int(m)
        except Exception:
            return 0


class ScheduleManager:
    """定时规则的管理器：负责 schedule.json 的读写与规则匹配。"""

    def __init__(self, path: str):
        self.path = Path(path)

    def load(self) -> list[ScheduleRule]:
        """从 schedule.json 加载全部规则（文件不存在/损坏时返回空列表）。"""
        if not self.path.exists():
            return []
        try:
            data = json.loads(self.path.read_text("utf-8"))
            return [ScheduleRule(**r) for r in data.get("rules", [])]
        except Exception:
            return []

    def save(self, rules: list[ScheduleRule]) -> None:
        """原子写入全部规则到 schedule.json。"""
        payload = {"rules": [r.__dict__ for r in rules]}
        tmp = self.path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp, self.path)

    def add(self, rule: ScheduleRule) -> list[ScheduleRule]:
        """新增一条规则并持久化。"""
        rules = self.load()
        rules.append(rule)
        self.save(rules)
        return rules

    def delete(self, rule_id: str) -> bool:
        """按 ID 删除规则；成功返回 True。"""
        rules = self.load()
        kept = [r for r in rules if r.rule_id != rule_id]
        if len(kept) == len(rules):
            return False
        self.save(kept)
        return True

    def set_enabled(self, rule_id: str, enabled: bool) -> bool:
        """启用/禁用某条规则。"""
        rules = self.load()
        for r in rules:
            if r.rule_id == rule_id:
                r.enabled = enabled
                self.save(rules)
                return True
        return False

    def active_rule(self, now: datetime | None = None) -> ScheduleRule | None:
        """返回当前时刻命中的第一条规则；无命中返回 None。"""
        for r in self.load():
            if r.active_now(now):
                return r
        return None

    def _ts(self):
        return time.time()
