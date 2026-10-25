# Go2-W SLAM 自主探索模块 — 设计与研发方案

> 版本：v1.0　日期：2026-10-25
> 面向对象：具身智能机器人领域系统架构/研发
> 安全基线：**默认不下发运动指令**，任何自主运动必须经操作员现场确认放行；越界即停；限速；一键急停；避障。
> 定位：在「安全固栏内」让机器人自主导航探索、边走边建图——**不是全自主漫游、不是导航调度、不是搬运**。

---

## 0. 一句话范围界定

本模块把现有「**被动的 SLAM 可视化**」升级为「**固栏约束下的主动边界探索**」：机器人沿探索边界（frontier）在虚拟围栏内以**低速档（≤0.3 m/s）**自主走到候选目标点，边走边建图；操作员在 2D 地图视图上**手工划定/编辑探索围栏**，围栏作为**安全硬约束**传入探索执行器。任何时刻操作员可一键急停，任何越界/跌落/碰撞/踏空/心跳失联都触发自动急停，然后回到「需要人工放行」的锁定态。

设计原则（贯穿全文）：

1. **运动放行是保守单点**：沿用已有的 `motion_enabled` 门禁 + `motion-gate`，并在探索执行器内增加**更窄的 `explore_armed` 硬闸**（只有操作员在探索页显式点「开始探索」才置位）。二者同时为真才允许下发速度。
2. **安全监督独立于规划**：探索策略（frontier 选点）与安全层（越界/限速/避障/急停）物理分离，安全层可独立于策略模块故障而停机。
3. **复用，不另起炉灶**：运动指令复用 `text_service` 的「安全监督 + 白名单限幅」链路与其动作原语；建图复用 `slam_worker`/`map_worker`；作业管理复用 `fleet_jobs`/`fleet_store`。

---

## 1. 模块定位与能力分层

把「SLAM 自主探索」拆成 8 个独立可验收子能力（每个都可单独测试、单独回滚）：

| 编号 | 子能力 | 职责 | 验收边界 |
|---|---|---|---|
| C1 | **探索状态机** | 探索作业生命周期：`idle→arming→arming`（机器人侧 wait）`→armed→exploring→paused→done/fault`；含暂停/恢复/取消/急停 | 状态迁移合法、可审计、崩溃可恢复 |
| C2 | **探索围栏编辑器（前端）+ 围栏模型（后端）** | 用户在 2D 地图上画多边形/矩形/圆形，顶点可编辑，序列化存储，作为安全硬约束 | 围栏形状合法性、面积/自交校验、越界判定正确 |
| C3 | **边界探索策略** | frontier-based 选点：从栅格地图提取前沿边界，贪心/最近边界+覆盖代价选目标 | 目标点可达、覆盖推进可量化 |
| C4 | **安全固栏与急停** | 越界判定、限速钳制、跌落/碰撞/踏空自动急停、操作员远程急停、心跳失联安全回退 | 越界 ≤某延迟停；急停响应 ≤300ms（端到端） |
| C5 | **局部避障** | 基于 LiDAR（`/api/lidar/obstacle`）前瞻制动/换向，走走停停 | 真机不碰撞、不损害环境 |
| C6 | **建图数据闭环** | 探索过程持续产出地图增量，前端实时叠加栅格/边界/位姿/目标 | 地图离线可看、闭环质量可核 |
| C7 | **前端可视化与操控面板** | 2D 地图叠加围栏/位姿/目标/边界/安全灯；一键急停大按钮；目标队列 | 交互可达、状态实时、急停大按钮恒可见 |
| C8 | **可测性与回放** | 探索步骤时间线、事件流、地图快照回放 | 事故可回溯、每一步有据可查 |

分层：C2/C7 前端+后端数据模型；C1/C3/C4/C5/C6 后端执行器；C8 贯通。

---

## 2. 探索策略设计

### 2.1 输入与表示

- **占用栅格**：复用 `slam_worker.py` 订阅 `/utlidar/grid_map`（PointCloud2，每点 `x,y,z,intensity=占用`）与 `/utlidar/robot_pose`（PoseStamped）已写好的 `slam_cache.json`（`grid` + `pose`，500ms 周期）。探索执行器**只读**该缓存，不与前端争抢写入。
  - 栅格点即 `occupied cell`；未知/自由由 frontier 逻辑推导。
- **位姿**：`slam_cache.json[i].pose`（`x,y,z`，不含航向）。**航向缺失是当前链路短板**——见风险 R2；短期用 Point-LIO `/odom_corrected`（`map_worker.py` 的 `PoseBuffer` 同源），探索执行器增加一个轻量订阅 `/odom_corrected` 以获得 `yaw`。
- **以机器人为原点**的局部代价/可达，仍用 `/api/lidar/obstacle` 的前方扇形最近点判定。

### 2.2 探索目标生成（frontier-based，最近边界 + 覆盖代价）

1. **栅格化**：把 `slam_cache.json.grid` 投影到 2D 平面（取 `x,y`，`z` 仅用于过滤 `z∈[−0.15,0.8]` 的落地障碍），量化到固定分辨率（如 0.1 m/cell）的 `occupancy grid`（三态：occupied / free / unknown）。
2. **前沿提取**：`frontier cell = free 且 8 邻域中至少 1 个 unknown 且未被占用`。用 BFS 把相邻 frontier cell 聚成 `frontier cluster`（阈值：cluster 面积 ≤ 某上限，避免一堵墙吞掉整条边界）。
3. **候选目标**：每个 cluster 取质心（或距离机器人最近的 cell）作为候选目标点 `candidate ∈ candidates`。
4. **选目标（贪心最近 + 覆盖代价）**：对每个 candidate 计算代价
   ```
   cost(c) = w1 · d(robot→c)            # 距离代价（鼓励近）
           + w2 · unknown_gain(c)⁻¹     # 信息增益（鼓励未知多的方向）
           + w3 · inv(recent_visit)     # 去重：近期去过则代价高
   ```
   取 `argmin cost`。默认权 `w1=1.0, w2=0.4, w3=0.3`；前端可调。
5. **目标内可达性预检**：对选中目标，用 `slam_cache.grid` 作障碍膨胀（机器人半径 + 0.1 m margin），做一次简单直线可达性/局部 A* 段校验；不可达则从候选中剔除重选（最多重选 3 次）。
6. **探索完成判定**：`remaining_frontier（经膨胀后的有效前沿 cell 数） < 阈值`（默认阈值按图幅设定，如 `< 20 cell` 或 `< 总图幅 1%`），或「连续 N 次无新前沿移动」→ 判定 `done`，状态机进入 `done`，机器人回中停止。

> 说明：本阶段策略中**不引入全局导航/重规划栈**（非导航调度）。目标点之间靠「速率命令 + 局部避障」逼近，而非路径规划器；这是与「导航调度」明确区分的边界。若未来需要，A* 仅作为**可达性校验**而非轨迹生成。

### 2.3 走走停停的状态转移（执行器内部）

用「短距逐步推进」代替连续漫游，每一步都落在安全门内：

```
可执行步骤 Step  = [ 感知(obstacle/grid/pose) → 安全校验(围栏/限速/姿态)
                    → 选目标(若需要) → 生成低速速度指令(幅度≤0.3m/s,时限≤0.5s)
                    → 等待并确认是否到位/是否被安全层打断 ]
```
每 0.5 s 重评估一次；任何安全层 `STOP` 会在指令生效下个周期内覆盖。

---

## 3. 安全体系设计

安全层采用**分层防御**，各层独立、任一触发即停机，恢复条件明确、保守。

| 层 | 触发条件（阈值可配） | 响应动作 | 恢复条件 |
|---|---|---|---|
| L1 **门禁层** | `motion_enabled=false`（robot.yaml）或 `explore_armed=false` | 不下发任何速度；探索作业保持 `arming`/`idle` | 操作员现场确认 + 页面点「开始探索」 |
| L2 **围栏层** | 机器人投影点（含 body 半径 + margin）在围栏外 / 距边界 < `fence_margin`（默认 0.2 m） | 立即 0 速，`fence_violation=true`，状态 `paused` | 操作员确认 + 重新放行 |
| L3 **限速层** | 期望速度模长 > `v_max_explore`（默认 0.3 m/s）或角速度超限 | 指令被钳制到上限；持续超限则 STOP | 命令恢复合法 |
| L4 **跌落/踏空层** | `foot_force` 任一腿 < `min_foot_force`（触地异常）OR `imu.pitch/roll` 超 `±40°`（姿态） | 立即急停 + 进入 `fault` + 警报 | 人工检查 + 复位 |
| L5 **碰撞层** | `/api/lidar/obstacle` 返回 `blocked=true`（前方 < `min_dist=0.6m`） | 立即 0 速，本级沿安全方向试探转向或 `paused` | 障碍清除 / 换目标 |
| L6 **操作员急停** | 页面大按钮 / `POST /api/explore/estop` | 立即 0 速 + `estop=true` + 门禁下电（`explore_armed=false`） | 人工复位 + 重新放行 |
| L7 **心跳失联** | 探索执行器 与 `collector`/`text_service`/固件 DDS 心跳超时（>1.5 s 无 fresh telemetry，复用 `telemetry_status` 新鲜度） | 立即 0 速 + `fault: heartbeat_lost`；**断链即停** | 链路恢复 + 人工放行 |

**关键设计**：L6 急停不是「发一条 0 速指令」这么弱——它同时把 `explore_armed` 置 false，使**即使 0 速指令丢失**，下一周期也因门禁关闭而不再生效。急停是状态性、可锁存的，不是一次性命令。

跌落判定数据源：`collector.py` 已采集且 `read_state()` 已聚合 `imu`（`roll_deg/pitch_deg/yaw_deg/quat`）、`foot_force`（12 处）、`sport_mode`。探索安全层复用这些字段，不新增传感器。

---

## 4. 架构与数据流

### 4.1 模块编排（复用/新增）

```
┌──────────── 前端 (web/index.html tv-slam tab) ────────────┐
│  2D Canvas 叠加：栅格/位姿/围栏/边界/目标点/安全灯/急停大按钮 │
└───────────────────────▲───────────────────────────────────┘
                        │ REST (见 §6)
┌───────────────────────┴───────────────────────────────────┐
│                   server.py  (FastAPI)                     │
│  新增 /api/explore/* 路由 → explore_api.py                 │
│  复用 /api/slam/map /api/lidar/obstacle /api/agent/motion-gate │
└──┬──────────────────┬──────────────────────┬──────────────┘
   │RPC               │读缓存                │写
   ▼                  ▼                      ▼
┌──────────────────────────────────────────────────────────┐
│  explore_executor.py (新增，核心)                          │
│   状态机 C1 + 策略 C3 + 安全 L2~L7 + 围栏 L2               │
│   - 读 slam_cache.json (grid/pose)                        │
│   - 读 /odom_corrected (yaw)  ← 新增轻量 ROS 订阅          │
│   - 读 /api/lidar/obstacle (复用，经 server 内部函数)       │
│   - 读 state.json imu/foot_force (复用 read_state())        │
│   - 速度指令 → 复用 motion 白名单通道 (见 4.3)              │
└──┬───────────────────────────────────────────────────────┘
   │ 写入 explore_jobs.json / explore_state.json (新增)
   ▼
┌──────────────────────────────────────────────────────────┐
│  复用 fleet_store.py (FleetStore) ● 新增 TEMPLATES['sl_explore'] │
│  复用 fleet_jobs.py (JobRunner) ● 新增 kind=='sl_explore' handler  │
└──────────────────────────────────────────────────────────┘
```

新增文件（写在 `go2w_monitor/` 下，与既有模块并列）：

1. **`explore_executor.py`**（核心）—— 类 `ExploreExecutor`，方法签名候选：
   ```python
   class ExploreExecutor:
       def __init__(self, slam_cache_path: Path, state_fn, obstacle_fn,
                    motion_gate_fn, fence_store: FenceStore, estop_evt: threading.Event)
       def start(self) -> None                      # 后台线程 loop
       def close(self) -> None
       def arm(self) -> None                        # 设置 explore_armed=true（须配合 motion_enabled）
       def disarm(self, reason: str) -> None        # 关闭 explore_armed
       def estop(self) -> None                      # 急停：0速 + disarm + estop=true
       def pause (self) / resume(self) -> None
       def loop(self): ...                          # 0.5s 周期：perceive→safety→plan→命令
       # 内部：
       def _load_grid(self) -> (np.ndarray, pose)   # 读 slam_cache.json
       def _extract_frontiers(self, grid) -> list[FrontierCluster]
       def _pick_target(self, frontiers, pose, cfg) -> ExploreTarget | None
       def _safety_check(self, ctx) -> SafetyVerdict  # L2~L7 汇总，任一 fail→STOP
       def _issue_velocity(self, v: float, w: float) -> None  # 经白名单通道，幅度钳制
   ```
2. **`explore_fence.py`**（围栏模型 + 校验 + 越界判定，纯几何，无 ROS、可单测）—— 类 `FenceStore` + `Fence` + `point_in_fence(px,py,fence,margin)`：
   ```python
   class Fence:
       kind: 'polygon'|'rectangle'|'circle'
       points: list[tuple[float,float]]   # polygon/rectangle 顶点；rectangle 存 2 对角点
       center: tuple[float,float]; radius: float   # circle
   class FenceStore:
       def load(self) -> list[Fence]; def save(self, fences) -> None
       def validate(self, fence) -> tuple[bool, str]   # 面积阈值/自交/最小顶点数
       @staticmethod
       def contains(self, fence, x, y, margin=0.0) -> bool  # 越界判定（含 edge margin）
   ```
   越界判定算法：多边形用**射线法（point-in-polygon）+ 对每条边计算点到线段距离 ≤ margin 即越界**；圆用 `dist(center) > radius - margin`。预校验：多边形顶点 ≥3、面积 ≥ `min_area`（默认 1 m²）、不自交（Shoelace 符号 + 边相交检测）、矩形宽高 ≥ 0.5 m、圆半径 ≥ 0.3 m。
3. **`explore_api.py`**（REST 路由，见 §6）。
4. **`explore_state.json` / `explore_jobs.json`**（运行时状态与作业持久化，`explore_fence.py` 同目录 json）。
5. **`control_schema.py` 扩展**（复用文件只增不改旧结构）：可选新增一个 `"explore"` 命令域，若走 control.json 通道则确保不破坏 collector 的既有 `start/stop` 解析。
6. **`web/explore-panel.js`**（前端新文件，或并入 `web/index.html` tv-slam 内联 + `web/fleet-context.js` 风格）。

复用点名（明确不改、只调用）：

- **`fleet_store.FleetStore`** 及其 `create_job/mutate/claim/cancel/event`，新增作业模板 `TEMPLATES['sl_explore']`（steps：`连接与状态检查`→`围栏校验`→`探索执行`→`建图与核验`）。
- **`fleet_jobs.JobRunner`** 的 tick 分支，新增 `kind=='sl_explore'` handler，把「探索推进」作为**有向无环步骤状态机**承接（job 状态驱动 executor 启停）。
- **`slam_worker.py` / `slam_cache.json`**：建图数据源（不透传、不改写，只读）。
- **`map_worker.py` / `map_geometry.PoseBuffer`**：位姿/航向同源参考（`/odom_corrected`）。
- **`read_state()` / `telemetry_status()`**（`server.py`/`recovery_policy.py`）：IMU、foot_force、心跳新鲜度。
- **`api_lidar_obstacle()` 内的前方扇形最近点逻辑**：局部避障判定（探索执行器在进程内直接复用该函数，避免跨 HTTP 引入延迟）。
- **`motion_enabled` 门禁 + `/api/agent/motion-gate`**：第一道放行闸（见 §4.3）。
- **`monitor_supervisor.py`**：只负责遥测/定位服务恢复，**不动**，也**不被探索执行器改为可恢复探索中 motion**——确保重启后探索永远回到 `arming` 等待人工放行（对齐「默认不下发运动」）。

### 4.2 数据模型

```jsonc
// explore_jobs.json —— 一条探索作业（挂在 fleet job 下，或独立 id）
{
  "id": "a1b2...",              // == fleet job id (可选)
  "robot_id": "go2w-01",
  "status": "queued|arming|armed|exploring|paused|done|fault|canceled",  // C1 状态机
  "fence_id": "f1",
  "fence": { "kind": "polygon",
             "points": [[x,y], ...],           // 世界/地图坐标
             "frame": "world_map" },           // 与 slam_cache grid 同参考系
  "params": { "v_max_explore": 0.3, "fence_margin": 0.2,
              "min_dist_obstacle": 0.6, "frontier_threshold": 20 },
  "progress": { "covered_cells": 1234, "total_frontier": 40,
                 "remaining_frontier": 12, "target_index": 3 },
  "targets": [ { "x":1.2,"y":0.8,"seq":3,"done":false }, ...],   // C7 目标队列
  "safety": { "estop": false, "fence_violation": false,
              "obstacle_blocked": false, "fall_detected": false,
              "heartbeat_ok": true },          // C4/C5 安全灯来源
  "steps": [ {"name":"连接与状态检查","status":"...","started_at":..,"finished_at":..} ],
  "events": [ {"at":..,"type":"armed|frontier_found|target_reached|estop|...","message":"..."} ],
  "created_at":..., "started_at":..., "finished_at":...
}
```

```jsonc
// explore_state.json —— 机器级瞬时状态（0.5s 刷新，前端高频轮询）
{
  "robot_id": "go2w-01", "online": true,
  "pose": {"x":..,"y":..,"z":..,"yaw":..},
  "safety": {"estop":false,"fence_violation":false,"obstacle_blocked":false,
             "fall_detected":false,"v_cmd":0.0,"w_cmd":0.0,"heartbeat_ok":true},
  "frontiers": [ {"x":..,"y":..,"cells":..}, ... ],      // 边界簇
  "target": {"x":..,"y":..}, "map_increment_ts": ...,
  "status": "exploring", "ts": ...
}
```

### 4.3 运动指令下发通道（复用 text_service 白名单，不另起炉灶）

现有运动链路为「文字→规划→SafetySupervisor→白名单限幅→动作 `move_distance/turn_angle`」，由 `motion_enabled` 门禁控制。探索执行器的速度指令**绝不复刻一套直连固件的发布器**，而是接入同一条**白名单监督通道**：

- 方案 A（首选，改动最小）：探索执行器把「目标点」翻译成 `text_service` 已有的**离散动作原语**（`move_distance` / `turn_angle`，步进小、带超时），每个原语前先过 `motion_enabled` + `explore_armed` 双门 + 围栏/避障预检。优点：天然复用 SafetySupervisor 的白名单限幅与急停语义；缺点：走离散动作、非连续速度。
- 方案 B（连续速度，需在白名单内新增一档）：在 `text_service` 的 SafetySupervisor 白名单里登记 `explore_velocity(v,w)`（幅度 ≤0.3 m/s、时限 ≤0.5 s），探索执行器经同一监督器下发。优点：更平滑；缺点：要动 `text_service` 白名单（仍在同一边界内，需在 `go2w_vla` 侧加档并留测试）。

设计落点：**阶段 1~2 用方案 A；阶段 3（固栏内自由探索）评估后迁到方案 B**。无论 A/B，两道独立闸（`motion_enabled`、`explore_armed`）都在探索执行器侧重入校验，不信任上游已查。

---

## 5. 前端页面设计（SLAM 探索 tab 改造）

沿用深空科技风（`#0e1f36` 底、`--border`/`--muted`/`--accent` 变量，徽章 `badge green/amber/red`），替换现有 `tv-slam` 区块为「**探索操控面板**」。

### 5.1 布局草图（ASCII）

```
┌───────────────────────────────────────────────────────────────┐
│  SLAM 自主探索   [● 状态: 探索中] [位姿:(1.2,0.8,0°)][刷新]     │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │                                                         │  │
│  │        ······························                    │  │
│  │      ·········███████···············   ← 已建栅格(占用)   │  │
│  │     ···█ obstacle ██·················                    │  │
│  │     ·████████████████···············   ← frontier 前沿(闪光)│
│  │        ······●robot·················    ● 机器人位姿      │  │
│  │              ···→ 目标队列            ▶ 当前目标点(闪烁)   │  │
│  │       ┌━━━━━━━━━━━━━━━━━┐                                │  │
│  │       ┃ 虚拟围栏(多边形) ┃  ← 可选中的顶点(可拖拽)           │  │
│  │       ┃  ●───────●      ┃                                │  │
│  │       ┃  │        │     ┃  绿色=安全 红色=越界告警         │  │
│  │       ┗━━━━━━━━━━━━━━━━━┛                                │  │
│  └─────────────────────────────────────────────────────────┘  │
│  [画笔多边形][矩形][圆形] [编辑顶点] [撤销] [清除]   [保存围栏]   │
│  ──────────────────────────────────────────────────────────   │
│  ● 安全灯: 门禁/围栏/限速/跌落/碰撞/心跳   (每灯 green/amber/red) │
│  状态: 探索中 ● 覆盖 61% 前沿 剩12/40  目标 #3/5 →(2.1,1.4)    │
│  ┌────────────┐  ┌────────────┐  ┌────────────────────────┐  │
│  │  [ ▶ 开始探索]│  │  [ ⏸ 暂停 ]│  │ [ ■ 急 停 ]  (超大红色)  │  │
│  └────────────┘  └────────────┘  └────────────────────────┘  │
│  (开始探索需二次确认 + 现场放行提示；急停恒可见/always-on-top)    │
└───────────────────────────────────────────────────────────────┘
```

### 5.2 交互说明（含围栏绘制工具的细化）

- **建图视图**：复用现有 `drawSlamGrid(grid, pose)`（`web/index.html`）的 canvas 绘制逻辑，并扩展叠加层：
  - 栅格点（强度→占用色，保留原）、机器人位姿（绿点，**新增航向箭头**用 `pose.yaw`）、边界前沿（闪光描边，来自 `/api/explore/state` 的 `frontiers`）、当前目标点（闪烁）、围栏（半透明填充 + 描边 + 可拖顶点）。
- **虚拟围栏编辑器**（新增 `web/explore-panel.js` 的 `FenceEditor`，复用 canvas 事件）：
  1. 三种绘制工具按钮：多边形（连续单击落顶点，双击/回车闭合）、矩形（拖拽对角）、圆形（拖拽圆心+半径）。
  2. **顶点编辑**：选中围栏后顶点渲染为可拖拽句柄（→ 鼠标按下拖拽改点；右键删除顶点）；拖动整条围栏平移。
  3. **即时合法性校验**：绘制/编辑中实时调用前端本地几何校验（多边形自交/面积），非法边以红色高亮并在「保存」时拦截（后端 `FenceStore.validate` 双保险）。
  4. **序列化**：点「保存围栏」→ `PUT /api/explore/fence`（见 §6），后端校验后存 `explore_jobs.json` 内 `fence` 字段，并回填 `fence_id`。前端从 `GET /api/explore/jobs/{id}` 拉回并回显。
- **探索进度**：覆盖百分比、前沿剩余、目标队列序号，读 `progress`/`frontiers`。
- **安全状态灯**：6 盏灯对应 L1~L6（门禁/围栏/限速/跌落/碰撞/心跳），读数来自 `/api/explore/state.safety`。
- **一键急停大按钮**：`POST /api/explore/estop`，固定 `position:fixed; bottom-right`，始终在页面（不随 tab 切换隐藏），点击后立即写入 `estop=true` + `explore_armed=false`。
- **开始探索**：点击后弹**二次确认**（「将下发运动指令，请确认现场无人/无障碍、机器人处安全位置」）+ 展示当前 `motion_enabled` 状态；确认后 `POST /api/explore/jobs` 创建 `sl_explore` 作业并 `arm`。

---

## 6. API 设计（新增/修改 REST 端点）

约定：$rid 默认 `go2w-01`；均返回 JSON；错误沿用 fleet 风格 `{detail, ...}`（HTTP 4xx/5xx）。路由注册在 `explore_api.py`，由 `server.py` `install_explore(app, BASE_DIR, read_state)` 挂载（对齐 `install_fleet`/`install_scenes` 模式）。

| 方法 | 路径 | 说明 | 请求/响应要点 |
|---|---|---|---|
| GET | `/api/explore/state` | 探索瞬时状态 | → §4.2 `explore_state.json` 内容 |
| GET | `/api/explore/jobs` | 探索作业列表（分页/按 $rid） | → `{jobs:[...], metrics}` |
| POST | `/api/explore/jobs` | 创建 `sl_explore` 作业 | body `{robot_id, fence, params?}`；返回 job JSON；**不自动 arm** |
| GET | `/api/explore/jobs/{jid}` | 单作业详情 | → 全量 `explore_jobs.json[jid]` |
| POST | `/api/explore/jobs/{jid}/arm` | 放行（开始探索） | 校验 `motion_enabled && fence.valid`；置 `explore_armed` |
| POST | `/api/explore/jobs/{jid}/pause` | 暂停（0 速但不 disarm） | |
| POST | `/api/explore/jobs/{jid}/resume` | 恢复 | 需围栏/安全健康 |
| POST | `/api/explore/jobs/{jid}/cancel` | 取消 | → `canceled` |
| POST | `/api/explore/estop` | 急停（全局/按 $rid） | 0 速 + `explore_armed=false` + `estop=true`；幂等 |
| GET | `/api/explore/fence` | 读当前围栏 | → `Fence` |
| PUT | `/api/explore/fence` | 保存/更新围栏 | `FenceStore.validate` → 存；非法 422 |
| DELETE | `/api/explore/fence` | 清空围栏 | 仅在非 `armed`/`exploring` 时 |
| GET | `/api/explore/frontiers` | 当前前沿簇（供前端叠加） | → `{frontiers:[...], threshold}` |
| GET | `/api/explore/map-increment` | 地图增量快照（如体积大用分块） | 对齐 `slam_cache`，供前端增量渲染 |

修改既有端点（仅增强，不破坏）：

- `GET /api/slam/map`：响应增补可选 `yaw`（若探索执行器已缓存 `/odom_corrected` 航向）与 `frontiers`/`target`，前端 `drawSlamGrid` 读取；无探索执行器时保持原样。
- `fleet_store.TEMPLATES` 增 `'sl_explore'`；`fleet_jobs.JobRunner.tick` 增 `kind=='sl_explore'` 分支，委托 `ExploreExecutor`。

心跳/失联：`GET /api/explore/state.safety.heartbeat_ok` 由探索执行器按 `telemetry_status()` 的新鲜度（1.5 s）计算；失联即 `false` + `status=fault`。

---

## 7. 分阶段实施计划（逐级放行）

每一阶段「不达标即回滚到上一阶段」，回滚条件明确。

### 阶段 0 — 仿真/规划可视化（**不动真机，无运动指令**）
- 交付：围栏编辑器 + 探索策略在**回放数据/仿真栅格**上跑，只产出 `frontiers/target`，**绝不 touch motion**。`explore_executor` 以 `dry_run=True` 运行，`_issue_velocity` 为 no-op。
- 验收：围栏可画/可存/可校验；frontier 选点可视化正确；覆盖曲线可看；`motion_enabled=false` 全程。
- 回滚：无副作用，直接停脚本。

### 阶段 1 — 限速真机小范围（离散动作，操作员贴身）
- 交付：启用方案 A（`move_distance/turn_angle`），`v_max_explore≤0.3`，`fence_margin=0.5`，面积 ≤ 2×2 m² 小围栏；操作员手持急停、身处急停可达范围。
- 验收：②跌落/碰撞/越界任一项触发即停；端到端急停响应 ≤300 ms；连续 5 min 无越界/无碰撞。
- 回滚：任一真机越界/碰撞/跌落即回切阶段 0 `dry_run`，复盘安全层后再试。

### 阶段 2 — 固栏内连续探索（方案 A/B 评估）
- 交付：逐步放宽围栏面积与 `fence_margin=0.2`；引入连续速度（方案 B，若白名单加档评审通过）与心跳失联回退全量启用。
- 验收：③探索覆盖率 ≥ 某阈值（如围栏内可进入 free 区域 90%）且 0 碰撞 0 越界；心跳断链即停。
- 回滚：覆盖率不达标或安全指标退化即回到阶段 1 参数。

### 阶段 3 — 多机器人 fleet 复用与回放
- 交付：`sl_explore` 纳入 `fleet_store` 作业模板，多机器人独立探索；步骤时间线 + 地图快照回放（C8）。
- 验收：④多机器人并行无资源互踩、链路互不干扰；事故可回溯。
- 回滚：单机器人降级即可。

---

## 8. 验收标准（阶段通过/失败判据）

| 编号 | 指标 | 通过判据 | 失败判据 |
|---|---|---|---|
| A | 探索覆盖率 | 围栏内 free 区 ≥90% 被覆盖 | <80% 且前沿仍显著（策略失效） |
| B | 无碰撞 | 全程 `/api/lidar/obstacle.blocked` 从不因接触触发（仅前瞻触发的正常制动不算） | 任何一次实际接触/损坏 |
| C | 急停响应 | 操作员急停→0速 ≤300 ms（端到端） | >500 ms 或急停后仍动 |
| D | 越界即停 | 任何越界在 ≤1 个控制周期（≤0.5 s）内 0 速 + `fence_violation=true` | 越界后继续运动 |
| E | 地图闭环质量 | 记录 `slam_cache` 增量，轨迹闭合误差 ≤ 图幅 2% | 显著漂移/丢帧导致地图撕裂 |
| F | 失联回退 | 心跳 >1.5 s 无 fresh → 0 速 + `fault` | 失联仍运动 |
| G | 可审计 | 每个状态迁移/急停/越界有 event + 时间戳 | 关键事件缺失 |

---

## 9. 风险清单

| # | 风险 | 等级 | 缓解 |
|---|---|---|---|
| R1 | **运动放行被误置真**（门禁逻辑/配置被绕过） | 高 | 双独立闸 `motion_enabled`+`explore_armed`；急停是锁存态；arm 必须二次确认 + 现场提示；`monitor_supervisor` 重启后永远回 `arming` |
| R2 | **航向缺失**（`robot_pose` 无 yaw）导致目标点相对方向错 | 中 | 探索执行器订阅 `/odom_corrected` 补 yaw；阶段 0 校验 yaw 可用性 |
| R3 | **围栏坐标与地图坐标不同参考系** | 高 | 围栏与 `slam_cache.grid` 同 `world_map` 参考系强制声明；阶段 0 用回放比对校准 |
| R4 | **下行指令与安全层竞态**（0 速指令丢失） | 高 | 急停 = 状态锁存非单次命令；0.5 s 周期重校验门禁 |
| R5 | **跌落/踏空漏检**（足底力阈值不适配轮式） | 中 | 阈值按轮式四足标定；叠 IMU 姿态阈值；阶段 1 小范围验证阈值 |
| R6 | **围栏自交/面积退化导致越界判定失效** | 中 | `FenceStore.validate` 前端实时 + 后端双保险；保存即校验 |
| R7 | **探索策略在局部小环/死胡同来回** | 低 | `recent_visit` 去重 + 前沿阈值判定 done + 步数上限 |
| R8 | **建图与探索争用 `slam_cache`/rospy** | 中 | 探索执行器只读缓存；独立 ROS node 订阅 `/odom_corrected`；写独立 `explore_state.json` |
| R9 | **多机器人隔离** | 低 | 每机独立 `explore_state`/`explore_jobs` 与 `robot_id` 作用域（fleet 已实现按 rid 作用域） |
| R10 | **操作员过度信任自主（人因）** | 高 | 默认不运动；开始探索强制现场确认文案；急停大按钮恒在；阶段逐级放行 |

---

## 10. 附：与既有模块的接口对照表（速查）

| 现有模块/文件 | 本设计如何复用 | 是否修改 |
|---|---|---|
| `slam_worker.py` / `slam_cache.json` | 只读建图数据源（grid+pose） | 否（只读） |
| `map_worker.py` / `map_geometry.PoseBuffer` | `/odom_corrected` 位姿/航向同源 | 否 |
| `collector.py`（imu/foot_force/sport_mode telemetry） | 跌落/姿态/心跳判定 | 否 |
| `server.py` `read_state()` / `recovery_policy.telemetry_status()` | 遥测新鲜度、心跳 | 否 |
| `server.py` `api_lidar_obstacle()` | 局部避障前方最近点 | 内部复用逻辑，进程内直调 |
| `server.py` `/api/agent/motion-gate` + `motion_enabled` | 第一道放行闸 | 读为主 |
| `text_service` SafetySupervisor + 白名单限幅 + `move_distance/turn_angle` | 运动指令下发（方案 A 复用；方案 B 加档） | 阶段 3 评估加档 |
| `fleet_store.FleetStore` / `fleet_jobs.JobRunner` | 探索作业状态机承接 | 增 `sl_explore` 模板 + tick 分支 |
| `fleet_api.py` `transport()`/`require()` | 多机作用域/远程转发 | 否 |
| `monitor_supervisor.py` | 服务恢复（**不动 motion**） | 否 |
| `web/index.html` tv-slam / `drawSlamGrid` | 前端复用绘制，扩展叠加层 | 修改 tab 内容 |
| `control_schema.py` `control.json` | 可选 explore 命令域（方案 A 可选通道） | 可选扩展 |

---

> 本文档为设计文档，不含实现代码；接口签名与文件清单精确到「新增什么文件、调用什么函数」粒度，供研发直接据此开发与评审。安全边界（默认不运动、越界即停、限速、急停、避障、心跳回退）为不可违反基线。
