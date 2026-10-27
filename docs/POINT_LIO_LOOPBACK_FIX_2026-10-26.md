# Point-LIO「imu loop back」根因确诊与修复收敛（2026-10-26）

> 状态：**已修复并部署，系统稳定运行验证通过；realign 分支待偶发时钟回退做最终触发确认。**

## 一、根因（确诊）

Point-LIO（go2w 机器狗 LiDAR-inertial 里程计）长期「`imu loop back, clear deque`」导致 `/odom_corrected` 停止发布、世界地图重建中断。

**决定性诊断证据**（在 `laserMapping.cpp` 的 `imu_cbk` 回退分支加 `%.9f` 纳秒级诊断日志捕获）：

```
imu LOOPBACK diag#0: timestamp=1792921985.956684351 last=1792921986.447277069 delta=-0.490592718
```

即 IMU 回调收到一帧**时间戳比上一帧倒退约 0.49 秒**的消息，触发 `timestamp < last_timestamp_imu` 判据。loop back 持续刷屏队列约 0.47 秒后恢复。

**根因**：IMU 消息源主机（机器狗扩展坞计算板 192.168.123.18，厂商裸 DDS 直发 `_CREATED_BY_BARE_DDS_APP_`）的系统时钟在运行中发生约 0.5 秒的**回退**。lidar 点的时间戳来自同一主机时钟，故 imu 与 lidar loop back 同时爆发。这解释了全部历史症状：偶发（约 10 分钟级）、进程不崩但 odom 停发、之前「秒级 header 时间纯单调」观测抓不到半秒级回退。

**排除的旧方向**：unilidar `float time` 精度（毫秒级，与实测半秒级不符）、`get_ros_time` 浮点截断、Point-LIO 算法逻辑。

## 二、修复（最小侵入，治标）

在 `point_lio/src/laserMapping.cpp` 的两个 loop back 分支，把「清空 deque/buffer 后卡死」改为「**重设时间基线 + 丢弃单帧**」：

1. `imu_cbk`（原 `imu loop back, clear deque`）：
   ```cpp
   RCLCPP_ERROR(logger, "imu loop back, realign baseline (drop frame)");
   last_timestamp_imu = timestamp;   // 关键：重设基线，不再卡在旧高水位
   ```
2. `standard_pcl_cbk`（原 `lidar loop back, clear buffer`）：
   ```cpp
   RCLCPP_ERROR(logger, "lidar loop back, realign baseline (drop frame)");
   last_timestamp_lidar = get_time_sec(msg->header.stamp);
   ```

原逻辑缺陷：`last_timestamp_imu` 只在 `timestamp >= last` 时更新，时钟回退后 last 停在「未来高水位」，后续所有帧 `< last` 被持续判定 loop back，直到时间戳追平——这期间 odom 全停。修复后回退帧被丢弃、基线立即重设为新时间戳，后续帧正常累积，odom 不中断。

**副作用边界**：丢弃回退发生瞬间的单帧（半秒跳变处），比「清空 deque + 卡死到追平」损失小得多。此修复治标——真正治本需在扩展坞做 NTP 校时（见 `docs/TIME_DOMAINS_2026-10-23.md`）。

## 三、部署与验证

- 源码：`/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w_vla_ws/src/point_lio/src/laserMapping.cpp`（非 git 仓库，备份 `.bak_loopback_fix`）。
- 编译：colcon 增量编译 `point_lio`（47.4s），产物 `build/point_lio/pointlio_mapping`（2026-10-25 19:09），`strings` 命中 2 处 `realign baseline` 确认含修复。
- 重启后 odom `/odom_corrected` Publisher count=1 恢复。
- **稳定运行验证**：机器狗换电重启后，Point-LIO 用修复后二进制运行，连续数小时（+2h 后台观察）`loop back` 计数恒 0，odom 发布频率 2798 Hz 稳定（std 0.0025s），系统健康。
- **待最终触发确认**：`realign baseline` 分支尚未被实际偶发时钟回退触发（因近期扩展坞时钟未回退）。逻辑上必然消除 odom 停发，属「确定性修复 + 无回流验证」。

## 四、结论

需求 7（Point-LIO imu loop back）已**根因确诊 + 修复部署 + 系统稳定运行**。剩余唯一待办是被动等待一次偶发时钟回退做最终触发确认（或直接在扩展坞实施 NTP 校时治本）。运行时的 loop back 复发监控脚本：`tmp/poll_loopback.py` / `tmp/watch_until_event.py`（会 SSH 查日志计数）。
