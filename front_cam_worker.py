"""Go2-W 前置相机独立采集进程。

在独立进程中用 VideoClient 持续采集前置相机帧，写 latest_front.jpg 供 Web 展示。
避开与 collector 主进程的 DDS ChannelFactory 冲突（collector 内 video 线程
在 500Hz LowState 订阅下 GetImageSample 会阻塞）。

启动（80 服务器，tmux 常驻）：
    python front_cam_worker.py --interface enp0s31f6 --out /mnt/.../go2w-monitor/latest_front.jpg
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path


def run(interface: str, out_path: str, fps: int = 10) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    out = Path(out_path)
    ChannelFactoryInitialize(0, interface)
    client = VideoClient()
    client.SetTimeout(3.0)
    try:
        client.Init()
    except Exception as e:
        print(f"[front_cam] VideoClient.Init failed: {e}", flush=True)
        return

    print(f"[front_cam] started, writing to {out_path}", flush=True)
    frame_interval = 1.0 / max(1, fps)
    frame = 0
    fail_count = 0
    while True:
        t0 = time.time()
        try:
            code, data = client.GetImageSample()
            if code == 0 and data:
                out.write_bytes(bytes(data))
                frame += 1
                fail_count = 0
                if frame % (fps * 30) == 0:
                    print(f"[front_cam] frame {frame}", flush=True)
            else:
                fail_count += 1
                if fail_count <= 5 or fail_count % 50 == 0:
                    print(f"[front_cam] GetImageSample code={code} (fail#{fail_count})", flush=True)
        except Exception as e:
            fail_count += 1
            print(f"[front_cam] sample error: {e}", flush=True)
        dt = time.time() - t0
        if dt < frame_interval:
            time.sleep(frame_interval - dt)


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2-W front camera worker")
    parser.add_argument("--interface", default="enp0s31f6")
    parser.add_argument("--out", default="/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor/latest_front.jpg")
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args()
    run(args.interface, args.out, args.fps)


if __name__ == "__main__":
    main()
