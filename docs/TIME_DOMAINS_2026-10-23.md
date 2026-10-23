# 时间戳差异排查与官方方案（2026-10-23）

本记录为在线只读诊断及方案核对，没有修改系统时钟、NTP 服务或雷达 SDK 连接。

## 实测

- 80 服务器：`NTP=yes`、`NTPSynchronized=yes`，systemd-timesyncd 正在运行，使用公网 NTP。
- 扩展坞 192.168.123.18：`NTP=yes`、`NTPSynchronized=no`；timesync-status 的 Server 地址为空、Packet count=0。NVIDIA drop-in 使用公网 pool 域名，当前未取得有效校时报文。不能把“服务 active”等同于“已校时”。
- 同次 SSH 检查中扩展坞 `date +%s` 约 175792，服务器约 1792742685，明显不在同一墙钟基准。扩展坞 uptime 约 703 秒；其 date 也不能直接当成开机单调计时。日志曾记录系统时钟回退，具体是谁调整时钟尚未确定。
- 20 秒 ROS 采样：雷达约 15.27 Hz、原始 IMU 约 250.80 Hz、适配后 IMU 约 251.30 Hz；四路源时间均无倒退。IMU 适配器深拷贝消息并保留 header，不重写时间戳。
- 适配后 IMU 与雷达最近源时间差中位数约 3.15 ms，里程计约 0.027 ms。这是观察窗口内消息匹配结果，不能证明曝光同步、雷达/IMU 固定延迟或全系统同步精度。
- IMU 源时间相对接收墙钟有约 8 ms 的“超前”中位值；需要进一步区分发布机校时误差、设备映射和传输排队，不能用平均接收延迟直接当作标定值。
- 原始 IMU 最近匹配最大值受采样起止边界与订阅建立先后影响，不用该最大值估计固定传感器偏移。
- RealSense API 返回 `timestamp_domain=2`，按官方枚举为 GLOBAL_TIME；该域转换到相机所在主机时钟，不代表主机间已经同步，也不代表曝光硬同步。

汇总见 `timestamp_audit_2026-10-23.json`，没有保存原始传感器载荷。

## 官方支持与适用范围

1. 宇树 L2 SDK 的初始化参数 `use_system_timestamp=true`，以及 `syncLidarTimeStamp()`，支持系统时间与雷达时间的处理。官方 ROS2 节点公开并传递该参数。
   - https://github.com/unitreerobotics/unilidar_sdk2/blob/main/unitree_lidar_sdk/include/unitree_lidar_sdk.h
   - https://github.com/unitreerobotics/unilidar_sdk2/blob/main/unitree_lidar_ros2/src/unitree_lidar_ros2/include/unitree_lidar_ros2.h
   当前订阅的是固件提供的 `/utlidar/*`，不是上述独立 SDK 节点，因此不能假定修改独立驱动参数就会改变现有固件话题；接入 SDK 前必须核对数据源与连接方式。
2. 宇树 Point-LIO L2 配置的 `time_lag_imu_to_lidar` 用于**经过标定**的固定偏移，IMU 时间减去该值后落入雷达时间线；`timestamp_unit=0` 表示秒。当前部署两项分别为 0.0 和 0。不能拿跨主机墙钟差填入传感器偏移。
   - https://github.com/unitreerobotics/point_lio_unilidar/blob/main/config/unilidar_l2.yaml
3. ROS 官方区分 SystemTime、SteadyTime 和 ROSTime；ROS 图中的计算机需同步系统时钟，仿真时间依赖 use_sim_time。超时/到达新鲜度用本地主机单调时间，跨设备事件比较需明确共同时间域。
   - https://design.ros2.org/articles/clock_and_time.html
4. RealSense GLOBAL_TIME 是设备时钟到所在主机时钟的转换；官方维护者说明它不根据曝光做同步调整。
   - https://realsenseai.github.io/librealsense/doxygen/rs__frame_8h_source.html
   - https://github.com/realsenseai/librealsense/issues/14732
   上述 issue 讨论 D555，其全局时间原理可参考，不能把其 DDS 版本修复直接当作当前 D435I 的解决补丁。
5. Chrony 支持 NTP server/client；systemd-timesyncd 支持用 drop-in 指定 NTP 服务器。
   - https://chrony-project.org/doc/4.6/chrony.conf.html
   - https://www.freedesktop.org/software/systemd/man/latest/timesyncd.conf.html

## 针对当前系统的实施顺序（尚未部署）

优先方案：80 服务器提供机器人网段内的局域网 NTP，扩展坞向 `192.168.123.222` 校时。80 服务器自身持续跟踪可信上游。扩展坞无需访问公网或依赖公网 DNS。

示例 Chrony 配置片段（不是已安装文件）：

```conf
# 80 服务器：选定可信上游后配置 server/pool，不要杜撰绝对时间
server <site-approved-upstream> iburst
allow 192.168.123.18/32
bindaddress 192.168.123.222
```

扩展坞 systemd-timesyncd drop-in 示例：

```ini
[Time]
NTP=
NTP=192.168.123.222
FallbackNTP=
```

先检查现有服务、UDP 123 可达性、系统包依赖及是否存在其他写时钟的固件进程；80 当前仅有 timesyncd 客户端，不能只把扩展坞 NTP 指向它就宣称完成。安装服务前先检查软件包更改清单，避免不必要替换共享服务器上的其他服务。

首次将扩展坞从异常时间校到当前时间会发生大幅跳变。应在机器狗静止、监控监督器与依赖该时钟的相机采集已停止的维护阶段执行，随后重启采集与融合，建立新的估计周期；不要在已有融合时间线中强改 date。先识别时钟反复回退的来源，否则仅配置 NTP 可能再次被其他进程覆盖。

验收必须分别确认：服务器/扩展坞 NTP 真正锁定、往返时差测量与抖动、ROS 节点 use_sim_time 一致性、源时间无倒退、相机跨主机时间语义、雷达点内时间单位。硬件/设备时间对齐需要再核对官方 SDK 在当前固件链路中的适用性；固定 IMU 延迟只有标定后才写入配置。

当前地图恢复验收可以继续运行，精确跨相机/雷达融合与导航时间同步仍不可标记为通过。
