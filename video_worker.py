"""相机采集线程（VideoWorker）。

在独立线程中轮询 Go2-W 前置相机（通过 unitree 的 VideoClient RPC 获取 JPEG 帧），
把每一帧保存为独立 JPG 文件到当前录制会话的 images/ 目录下，并与状态录制保持
时间上的同步（状态帧通过 image_path 列引用最近的图像文件）。

注意：DDS 的 ChannelFactory 是单例，由 collector 主线程先初始化，相机线程
复用该工厂而不是重复初始化，否则会导致冲突崩溃。
"""

from __future__ import annotations

import threading
import time
from pathlib import Path


class VideoWorker:
    """前置相机采集器。"""

    def __init__(self, interface: str, dataset_root: str, fps: int = 15):
        self.interface = interface              # 机器人专网网卡
        self.images_root = Path(dataset_root) / "sessions"  # 图像存储根目录
        self.fps = fps                          # 目标采集帧率
        self._stop = threading.Event()          # 停止信号
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()           # 保护共享状态
        self._latest_path: str | None = None    # 最近一帧图像路径
        self._seq = 0                           # 帧序号
        self._session_id: str | None = None     # 当前录制会话
        self._frame_interval = 1.0 / max(1, fps)  # 帧间隔（秒）

    def start(self) -> None:
        """启动采集线程（幂等）。"""
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止采集线程。"""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    def set_session(self, session_id: str | None) -> None:
        """切换当前录制会话；None 表示停止写入图像。"""
        with self._lock:
            self._session_id = session_id
            self._seq = 0
            self._latest_path = None

    def latest_image_path(self) -> str | None:
        """返回最近一帧的图像文件路径（供状态帧引用）。"""
        with self._lock:
            return self._latest_path

    def _loop(self) -> None:
        """采集主循环：轮询 VideoClient 获取 JPEG 帧并落盘。"""
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient
        except Exception as e:
            print(f"[video] init failed: {e}", flush=True)
            return

        # DDS ChannelFactory 已由主线程初始化；VideoClient 复用该单例工厂
        client = VideoClient()
        client.SetTimeout(3.0)
        print("[video] creating client...", flush=True)
        try:
            client.Init()
            print("[video] Init done", flush=True)
        except Exception as e:
            print(f"[video] VideoClient.Init failed: {e}", flush=True)
            return

        print("[video] camera worker started", flush=True)
        fail_count = 0
        while not self._stop.is_set():
            t0 = time.time()
            try:
                code, data = client.GetImageSample()
                if code == 0 and data:
                    self._save_frame(bytes(data))   # 保存 JPEG 帧
                    fail_count = 0
                else:
                    fail_count += 1
                    if fail_count <= 5 or fail_count % 50 == 0:
                        print(f"[video] GetImageSample code={code} len={len(data) if data else 0} "
                              f"(fail#{fail_count})", flush=True)
            except Exception as e:
                fail_count += 1
                print(f"[video] sample error: {e}", flush=True)
            # 按目标帧率限流
            dt = time.time() - t0
            if dt < self._frame_interval:
                time.sleep(self._frame_interval - dt)

    def _save_frame(self, jpeg: bytes) -> None:
        """保存 JPEG 帧。

        - 始终写入独立的最新帧文件 latest_front.jpg（供 Web 实时预览，不依赖录制）
        - 录制会话期间，额外保存到该会话的 images/ 目录，并让 _latest_path 指向
          会话内帧文件（供状态帧引用，标注/帧浏览时能正确找到图像）
        """
        with self._lock:
            self._seq += 1
            seq = self._seq
            # 1) 始终保存最新帧到独立文件（实时预览用）
            latest_path = self.images_root.parent / "latest_front.jpg"
            try:
                latest_path.write_bytes(jpeg)
            except Exception as e:
                print(f"[video] latest save error: {e}", flush=True)
            # 2) 录制会话期间，保存到会话目录，并把 _latest_path 指向会话帧文件
            session_id = self._session_id
            if session_id:
                session_images = self.images_root / session_id / "images"
                session_images.mkdir(parents=True, exist_ok=True)
                path = session_images / f"frame_{seq:06d}.jpg"
                try:
                    path.write_bytes(jpeg)
                    self._latest_path = str(path)   # 供状态帧引用会话内图像
                except Exception as e:
                    print(f"[video] save error: {e}", flush=True)
                    self._latest_path = str(latest_path)
            else:
                self._latest_path = str(latest_path)
