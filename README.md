# Go2-W 遥测监控与数据集构建系统

面向 Unitree Go2-W 轮式四足机器人的**实时遥测监控 + 具身智能训练数据集构建**一体化系统。

系统运行于机器人部署服务器（Ubuntu 22.04 + ROS 2 Humble），通过 DDS 订阅 Go2-W 的 500Hz 低频状态和 10Hz 运动模式状态，提供 Web 端的实时监控、数据录制、自动采集、自然语言标注，并支持导出为 HuggingFace 与 LeRobot 标准训练格式，服务于具身智能持续学习（VLA）数据流水线。

---

## ✨ 核心功能

| 功能 | 说明 |
|------|------|
| 📡 **实时监控** | 500Hz 状态流实时可视化：IMU 姿态、电压/电流、电池电量、12 关节电机、足底力、运动模式，带实时曲线图 |
| 🎥 **相机同步** | 前置相机以 ~15FPS 采集 JPEG 帧，与状态帧按时间对齐（`image_path` 引用） |
| 📷 **深度相机** | Intel RealSense D435I 深度相机（扩展坞计算板），RGB 彩色图 + 伪彩色深度图实时展示，独立采集程序绕过有 bug 的 ROS 节点 |
| 🛰 **激光点云** | 激光雷达点云 3D 实时查看（Three.js），可旋转缩放，点数/刷新频率可调 |
| 🎬 **数据录制** | Web 一键开始/停止录制，流式写盘（任意时长不占内存），自动崩溃恢复 |
| ⏰ **定时采集** | 按星期 + 时间段自动开始/停止录制，无人值守持续积累数据，支持跨天时段 |
| 🏷 **标签标注** | 会话级标签 + **帧级自然语言标注**（选择单帧/多帧 → 写详细语言描述） |
| 🚀 **数据集导出** | 一键导出 **HuggingFace** 或 **LeRobot v2** 标准训练格式（含视频 MP4） |
| 🖥 **现代化 Web UI** | 深空科技风界面，玻璃态卡片 + 霓虹点缀，ECharts 图表 + 四大工作台 TAB |

---

## 🏗 系统架构

```
┌────────────────────────────── Go2-W 机器人 (192.168.123.161) ─────────────────────────────┐
│                                                                                            │
│   DDS 状态流 (500Hz)          DDS 运动模式 (10Hz)         VideoClient RPC (~15FPS JPEG)     │
└───────┬──────────────────────────────┬──────────────────────────────┬─────────────────────┘
        │                              │                              │
        ▼                              ▼                              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                                collector.py (采集进程, tmux 常驻)                             │
│  ┌─────────────┐  ┌────────────────┐  ┌──────────────┐  ┌─────────────────────────┐        │
│  │ DDS 订阅     │→ │ 状态解析/快照    │→ │ InfluxDB 写入 │→ │ 录制引擎 (Arrow 流式)     │        │
│  │ LowState     │  │ state.json     │  │ (降采样)      │  │ recorder.py + camera    │        │
│  │ SportMode    │  └────────────────┘  └──────────────┘  └─────────────────────────┘        │
│  └──────┬──────┘     ▲        ▲        ▲                    ▲                              │
│         │            │        │        │                    │                              │
│         │     ┌──────┴────────┴────────┴──────┐        ┌────┴──────────────┐               │
│         └─────│   control.json  命令通道        │        │ schedule.json 定时 │               │
│               │  (Web→collector)              │        │ (自动录制规则)     │               │
│               └───────────────────────────────┘        └───────────────────┘               │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
        │                             ▲
        │ state.json / control.json   │ REST API
        ▼                             │
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                              server.py (FastAPI :8000, tmux 常驻)                            │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────┐ ┌─────────────────────────────────┐      │
│  │ 实时状态 API  │ │ 录制控制 API  │ │ 定时规则 API  │ │ 帧浏览 + 标注 + 数据集导出 API     │      │
│  └─────────────┘ └──────────────┘ └──────────────┘ └─────────────────────────────────┘      │
└───────────────────────────────┬─────────────────────────────────────────────────────────────┘
                                │
                                ▼
                    ┌───────────────────────────┐
                    │  Web UI (index.html)       │
                    │  实时监控 / 数据录制 / 视觉与 │
                    │  点云 / 标注 工作台          │
                    └───────────────────────────┘
```

### 深度相机架构（RealSense D435I）

Go2-W 扩展坞计算板（192.168.123.18，aarch64）上运行独立的 `rs_capture` 采集程序：

```
机器狗扩展坞计算板 (192.168.123.18, ARM64)
├── Intel RealSense D435I  (USB, /dev/video0-5)
├── librealsense2 2.57.7    (驱动库)
├── rs_capture             (独立采集程序, C++/librealsense API)
│   ├── 彩色帧 → latest_color.jpg  (640x480 JPEG)
│   └── 深度帧 → latest_depth.jpg  (伪彩色, 近红远蓝)
└── 输出目录: /home/unitree/rs_out/

80 服务器 (server.py)
├── /api/depth/color  → 从扩展坞拉取最新彩色图
├── /api/depth/image  → 从扩展坞拉取最新深度图
└── /api/depth/status → 深度相机状态

浏览器
└── 视觉与点云 TAB → 深度相机 RGB + 深度图双屏实时展示
```

> **为什么不用 ROS 节点？** 官方 `realsense2_camera` ROS 节点在此环境有 `std::bad_alloc` bug
> （ROS 包编译用的 LibreRealSense 2.51 与系统运行时 2.57 ABI 不兼容）。相机硬件本身完全正常，
> 独立 C++ 采集程序（`dock/rs_capture.cpp`）绕过 ROS 直接用 librealsense API 采集，稳定可靠。

### 数据存储布局

```
<dataset-root>/                          # 默认 /mnt/<big-disk>/go2w-dataset
├── schedule.json                        # 定时录制规则
├── _in_progress/                        # 录制中的临时目录（崩溃可恢复）
├── sessions/
│   └── <session_id>/
│       ├── raw.arrow                    # 全量状态帧（Arrow IPC，100 列）
│       ├── images/                      # 同步相机帧（frame_000001.jpg ...）
│       ├── metadata.json                # 会话元信息（时长/样本/标签等）
│       └── annotations.json             # 帧级自然语言标注
└── exports/
    ├── <session_id>/                    # HuggingFace 格式
    └── <session_id>_lerobot/            # LeRobot v2 格式
```

---

## 🛠 快速开始

### 环境要求

- **机器人侧**：Unitree Go2-W（DDS 专网可达）
- **服务器侧**：Ubuntu 22.04、ROS 2 Humble、Python 3.10
- **依赖库**：

```bash
# 创建并激活虚拟环境（含 system-site-packages 以复用 ROS 库）
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

pip install fastapi uvicorn requests pyarrow
# 可选：导出视频用
pip install opencv-python-headless
```

### 配置

修改各文件顶部常量（`server.py` 与 `collector.py` 的默认值）：

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `interface` | `enp0s31f6` | 机器人专网网卡名 |
| `influx-url` | `http://127.0.0.1:8091` | InfluxDB HTTP 端口 |
| `database` | `go2w_monitor` | InfluxDB 数据库名 |
| `dataset-root` | `/mnt/.../go2w-dataset` | 数据集根目录（建议放空间充足的大磁盘） |

### 启动服务（tmux 常驻）

```bash
# 1. 启动采集进程（含相机）
tmux new-session -d -s go2wcol "python collector.py --interface enp0s31f6 --database go2w_monitor --camera"

# 2. 启动 Web 服务
tmux new-session -d -s go2wweb "python server.py"
```

访问 `http://<服务器IP>:8000` 打开监控页面。

> 若不需要相机，省略 `--camera` 参数即可。

---

## 📖 使用指南

### 实时监控
顶部导航 →「实时监控」：查看 IMU 姿态曲线、电压/电流、电池电量、12 关节电机表、足底力、运动模式。

### 数据录制
顶部导航 →「数据录制」：
1. 在「标签」输入框填写会话标签（选填，如 `walking`）；
2. 点击「▶ 开始录制」，机器人状态开始流式写盘（相机同步采集）；
3. 点击「■ 停止录制」，会话保存到 `sessions/<id>/`；
4. 会话列表可选择导出为 **HF** 或 **LeRobot** 格式。

### 定时采集
「数据录制」面板下方的「定时录制」：
1. 选择星期 + 开始/停止时间 + 标签；
2. 点击「+ 规则」添加；collector 每秒检查规则，到点自动开始/停止。
3. 支持跨天时段（如 23:30 → 00:30）与多条规则并存。

### 帧级自然语言标注
顶部导航 →「标注工作台」：
1. 左侧选择会话；
2. 中间浏览帧缩略图（显示相机图 + tick + IMU 姿态），点击选中单帧或多帧（跨页累计）；
3. 在文本框中写自然语言描述（如"机器人以稳定步态向前行走，右前腿支撑"）；
4. 点击「保存标注」，标注持久化到 `annotations.json`；
5. 导出 LeRobot 时，标注自动写入 `tasks.jsonl`，对应帧设 `task_index`。

---

## 🚀 数据集导出

### HuggingFace 格式
```
<export>/<session_id>/
├── dataset_info.json          # schema + split 信息
└── data/
    ├── train/data-00000-of-00001.parquet
    └── test/data-00000-of-00001.parquet
```
可用 `datasets` 库直接加载：
```python
from datasets import Dataset
d = Dataset.from_parquet("<export>/data/train/data-00000-of-00001.parquet")
```

### LeRobot v2 格式（推荐用于 VLA 训练）
```
<export>/<session_id>_lerobot/
├── meta/
│   ├── info.json              # 数据集 schema（fps/features/维度）
│   ├── episodes.jsonl         # episode 元信息
│   ├── tasks.jsonl            # 自然语言标注任务（若存在）
│   └── stats.json             # 归一化统计（mean/std/min/max）
├── data/chunk-000/
│   └── episode_000000.parquet # observation.state(70D) + action(24D) + task_index
└── videos/chunk-000/
    └── observation.images.camera/episode_000000.mp4
```

`observation.state` 70 维：IMU(四元数+陀螺仪+加速度+欧拉角) + 电源/电池 + 12 电机(q/dq/tau/temp) + 4 足底力。
`action` 24 维：12 电机角度 + 12 电机速度的帧间增量（delta）。

---

## 📁 项目结构

```
go2w_monitor/
├── collector.py          # 采集进程：DDS 订阅 + 状态解析 + InfluxDB + 录制调度
├── server.py             # FastAPI 后端：REST API（状态/录制/定时/标注/导出/深度/点云）
├── recorder.py           # 录制引擎：Arrow IPC 流式写盘 + 崩溃恢复
├── dataset_builder.py    # 数据集导出：HuggingFace + LeRobot 两种格式
├── scheduler.py          # 定时调度：自动录制规则管理
├── control_schema.py     # 控制文件协议：collector 与 server 的命令通道
├── video_worker.py       # 相机线程：VideoClient 轮询 + JPEG 落盘
├── dock/
│   ├── rs_capture.cpp    # 深度相机采集程序（运行于机器狗扩展坞，librealsense C++ API）
│   └── build.sh          # 深度相机采集程序编译脚本（机器狗 aarch64）
└── web/
    ├── index.html        # 前端：深空科技风单页应用（四大工作台 TAB）
    └── lib/              # 前端库：echarts / three.js(r128 UMD) / OrbitControls
```

---

## 🔧 运维与排障

### 服务器重启后恢复服务
```bash
# 重新启动两个 tmux 会话（见上方「启动服务」）
tmux new-session -d -s go2wcol "python collector.py --interface enp0s31f6 --camera"
tmux new-session -d -s go2wweb "python server.py"
```
> 建议将上述命令写入 systemd 服务实现开机自启。

### 深度相机（RealSense）采集程序
深度相机在机器狗扩展坞计算板（192.168.123.18）上运行独立采集程序：
```bash
# SSH 到扩展坞
ssh unitree@192.168.123.18   # 密码 123

# 编译（源码见 dock/rs_capture.cpp）
cd /home/unitree/rs_dock && bash build.sh

# 启动采集（输出到 /home/unitree/rs_out，持续运行）
nohup ./rs_capture /home/unitree/rs_out 640 480 15 < /dev/null > /tmp/rs_cap.log 2>&1 &
```

**常见问题**：
- 深度相机不识别：重启机器狗让扩展坞重新枚举 USB（`/dev/video*` 会出现，`lsusb` 见 `8086:0b3a`）；
- ROS 节点 `realsense2_camera` 有 `std::bad_alloc` bug（librealsense 版本 ABI 不匹配），**使用独立采集程序 rs_capture 绕过**；
- 深度图伪彩色含义：近处偏红(暖)，远处偏蓝(冷)，显示范围 0.2m~4m。

### 采集进程崩溃
- `collector.py` 崩溃后，录制中的 `_in_progress/` 残留会在下次启动时自动回收（`recover_crashed`），半成品会话被标记 `crashed=True`。
- 常见崩溃原因与对策：
  - **DDS 初始化失败**：确认网卡名正确、`source /opt/ros/humble/setup.bash` 已执行；
  - **相机线程冲突**：确保 DDS `ChannelFactoryInitialize` 在主线程先于相机线程执行；
  - **日志权限**：`collector.log` 若为 root 所有导致 tee 失败，先 `sudo chown dell:dell collector.log`。

### 常用 API
| 方法 | 路径 | 用途 |
|------|------|------|
| GET | `/api/health` | 服务器 + 机器人在线状态 |
| GET | `/api/state` | 最新状态快照 |
| POST | `/api/record/start?label=xxx` | 开始录制 |
| POST | `/api/record/stop` | 停止录制 |
| GET | `/api/sessions` | 会话列表 |
| GET | `/api/session/<id>/frames` | 帧浏览 |
| POST | `/api/session/<id>/annotations` | 添加自然语言标注 |
| GET | `/api/dataset/export/<id>` | 导出 HuggingFace |
| GET | `/api/dataset/export/lerobot/<id>` | 导出 LeRobot |

---

## 📄 License

MIT License — 详见 [LICENSE](LICENSE)。
