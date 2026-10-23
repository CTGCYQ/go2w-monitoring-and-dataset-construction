# 监控系统恢复与稳定性验收（2026-10-22～23）

范围：第 4 项长时运行、资源占用、通信中断重连与机器狗关机重启恢复。所有脚本仅处理监控、传感器及定位进程；没有下发机器狗运动指令，运动门禁继续关闭。

## 已部署的修改

- 增加 `monitor_supervisor.py`，由用户级 `go2w-monitor.service` 管理；已启用服务和 `loginctl enable-linger dell`，无需 SSH 会话常驻。
- 接管健康的既有 tmux 进程；监控 Web、雷达、IMU 适配器、遥测和前置相机，异常退出后按 3～60 秒退避重启。Web 连续健康探测失败也触发恢复。
- 雷达中断后停止旧 Point-LIO 和建图进程；至少 3 个不同新帧且持续稳定 3 秒后，启动新的定位与地图周期。历史地图保留；未知坐标/几何故障继续锁定，不以重启掩盖故障。
- 修复 tmux 已启动但 ROS/Python 子进程尚未出现时被误判失败的竞态，等待实际子进程启动。
- 遥测缓存超过 3 秒不能继续显示在线；前置图像超过 3 秒返回 503，实时接口不再用历史录制帧顶替。
- 前置 JPEG 原子替换，连续取帧失败让监督器重建客户端。RealSense 在线状态依据内存中的新深度帧，SFTP 图像要求文件持续更新并确保异常路径关闭连接。
- 补齐部署端缺失的 DepthReceiver 初始化及原始深度路由；实时深度已复测。
- 新增 `/api/system/health`：监督器状态新鲜度、进程 PID、重启次数、地图周期及体素数量。状态过期不会继续报告监督器在线。
- 监督日志及各工作进程输出有大小上限与轮转。

## 已完成验收

57 项后端单元测试通过。最终部署版本再次完成 8 项故障恢复：

| 故障进程 | 恢复时间（秒） |
| --- | ---: |
| Web | 1.601 |
| 雷达采集 | 2.156 |
| 建图 | 2.112 |
| Point-LIO | 4.738 |
| IMU 适配器 | 6.907 |
| 前置相机 | 0.586 |
| 遥测采集 | 2.132 |
| 监督器 | 3.760 |

监督器恢复时复用健康工作进程，无重复实例；浏览器保持原页面，无需刷新，没有浏览器错误。详见 `reliability_process_2026-10-23.json`。

10 月 22 日完成真实机器狗关机、开机验收：离线时旧点云隐藏、旧定位停止、深度离线、实时相机返回 503；新雷达帧恢复后约 27.18 秒，相机、深度与地图恢复，同一浏览器页面无需刷新。本轮发现了启动竞态，之后已修复。详见 `reliability_power_cycle_2026-10-22.json`。

10 月 23 日在最终部署版本上进行 15 秒机器人网络隔离：只丢弃机器人/扩展坞两个 IPv4 地址的数据，80 服务器管理网络不变；清理定时器先于隔离启动，测试后确认临时规则全部移除。离线保护与新定位周期均通过，新雷达帧恢复后约 5.84 秒全面恢复。详见 `reliability_network_2026-10-23.json`。

昨日物理断线观察因等待超时没有得到完整报告，不计作通过；上述网络隔离验收也不等同于物理拔线验收。

## 连续运行验收

用户明确选择将本轮验收缩短为 10 分钟，30 分钟长测留作后续回归。实际持续 602.783 秒，共 61 个样本，全部健康；源帧推进 60 次，工作进程 PID 全程一致，浏览器/着色器/API 实际错误为 0。

GC 后浏览器堆由 13.11 MiB 到 14.13 MiB，增加约 1.02 MiB；几何体数量 7～8、纹理固定 3。Web RSS 122.20→148.07 MiB，建图 75.70→93.64 MiB，Point-LIO 255.45→258.98 MiB；其余各进程增加不超过 1.85 MiB。FD 无持续增长。CPU 指标以单核 100% 计，建图平均约 113.75% 表示约 1.14 个核，不是整机超载百分比。上述结果满足本次资源阈值，不能外推长期无泄漏。

详见 `reliability_soak_10min_2026-10-23.json`。原始运行器按 1800 秒目标启动，用户改为 600 秒后通过 stop 文件结束；原运行器因此报告“未达到原目标”，报告中保留该原始判定，并单独按用户授权的 10 分钟窗口验收，没有把 30 分钟标记为完成。

测试每 10 秒观察 API、雷达源帧推进、地图状态、各进程 RSS/FD/线程/CPU 与 WebGL 资源；定期在实时帧、时间聚合、地图视图间切换，并采集 GC 后堆占用。报告仅保存汇总指标，不保存点云、原始深度或房间截图。

## 启动与维护

80 服务器上的部署目录：`/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor`。

以 dell 用户在服务器执行：

```sh
export XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
systemctl --user status go2w-monitor.service
systemctl --user start go2w-monitor.service
journalctl --user -u go2w-monitor.service -n 50 --no-pager
```

手动启动整个监控由 `systemctl --user start go2w-monitor.service` 完成，网页仍为 `https://192.168.1.80:8000`。重启监督器用 `systemctl --user restart go2w-monitor.service`，它会接管健康工作进程。

维护时先 `systemctl --user stop go2w-monitor.service`，再停止/替换指定工作进程，否则监督器会恢复它们。单位使用 `KillMode=process`：停止监督器不会自动关闭已有 tmux 工作进程，也不会杀死其他研发服务。修改某个工作进程代码后需单独重启该进程，再启动监督器。

首次部署的环境依赖及顺序：已有 VLA `.venv`、ROS Humble/CycloneDDS 和机器人网卡 `enp0s31f6`；安装本次 Python 模块与 unit，`systemctl --user daemon-reload`、`systemctl --user enable --now go2w-monitor.service`，管理员执行一次 `sudo loginctl enable-linger dell`。unit 中部署目录及 VLA 路径是当前服务器固定路径，迁移时需修改。

复测脚本：`tools/test_recovery_live.py` 会有意终止监控工作进程；`tools/test_robot_reconnect.py` 观察操作员关机/开机或受控断网；`tools/test_monitor_soak.py --seconds 1800 --output report.json` 持续运行。涉及 SSH 的测试从环境变量 `GO2W_SSH_PASSWORD` 读取密码，报告不包含凭据。Windows 浏览器验收脚本使用当前已安装 Chrome 路径及 Playwright。

## 边界

- 80 服务器配置了自动启动及用户 linger，但没有重启这台共享研发服务器，因此不宣称通过服务器整机断电验收。
- 本轮只完成 10 分钟资源验收，30 分钟长测尚未完成；短时结果不代表无限运行无泄漏。
- 精确尺度、回程误差、完整机身外参及导航 TF 验收不属于本项，运动/导航能力仍未开放。

时间域排查另见 [时间戳诊断与官方方案](TIME_DOMAINS_2026-10-23.md)。扩展坞未完成 NTP 校时的问题已确认，校时方案尚未部署，不计作已修复。
