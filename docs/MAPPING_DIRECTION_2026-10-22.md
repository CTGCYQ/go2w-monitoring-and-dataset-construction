# Go2-W 方向与世界地图修复验收（2026-10-22）

显示方向、世界地图计算、当前 Go2-W 固件的 IMU 适配与持久启动配置已部署。入口：https://192.168.1.80:8000/ → 视觉与点云，Ctrl+F5 更新至资源版本 20261022-5。

## 地图计算

旧地图首帧使用绝对坐标，后续帧使用不同定义的相对坐标，相对旋转顺序也有误；缺少原点偏移、源时间匹配、帧去重和内存边界。旧缓存出现约 -327km 的 Z，已保留原备份，未用平移缩放掩盖异常。

新实现统一采用初始 IMU 参照：R_rel=R_anchor^T R_current，t_rel=R_anchor^T(t_current-t_anchor)，首帧也遵循这一关系。雷达点先变到 IMU：p_I=p_L+[0.007698,0.014655,-0.00667]。内部轴/平移定义来源：[Unitree L2 SDK2](https://github.com/unitreerobotics/unilidar_sdk2#2-coordinate-system-definition)，不等同于机身安装 TF。

按 PointCloud2 的逐点 time（秒）匹配位姿，线性插值位置、SLERP 插值旋转，实现逐点运动补偿。1ms 计算分组内时间差小于 1ms。拒绝位姿外推、超过 0.1s 的插值间隙、未知点时间、错误 frame、非有限数据及异常范围。不声称硬件时钟已独立标定。

位姿最多 2500 样本，待处理点云最多 8 帧，地图最多 120000 体素/10000 轨迹点，后端默认 6cm 体素。帧去重；不大于 10ms 的乱序位姿丢弃并计数，更大时间回退、断流或异常位姿跳变锁存 fault。原始时间戳不修改，数据过期转历史画面，异常明确冻结。API 再检查缓存写入新鲜度及全部点/轨迹的有限性、合理范围。

世界地图显示完整受限地图，不再以距离雷达原点裁剪远处区域。显示体素与后端体素是不同降采样层，均不代表绝对测量精度。

## IMU 和方向证据

用户确认机器狗自身左转。机身独立 IMU 历史 yaw 约从 -3.29° 到 +43.96°，而原始雷达与旧显示里程计呈负向旋转。近点云 3113 点中，最大近水平面约 2097 内点，位于源 +Z 约 0.463m；静止雷达加速度与指向该平面的源方向大致同向。这些现场证据用于提出并验证适配假设，不推广到任意 L2 SDK 或其他固件。

lidar_imu_adapter.py 将本次已验证固件的重力式加速度取反，转换为 specific force，发布至 /go2w/lidar_imu_specific_force；陀螺仪、时间戳、frame 保留，orientation 标为未知。原始 DDS IMU 与点云不改写。Point-LIO 改用适配话题，不发布机身静态 TF。

适配后的 30 秒静止净漂移 0.00852m、最大偏移 0.02527m。用户确认前进，记录最大约 0.6792m、横向偏离约 0.0516m；以运动向量和静止 specific force 计算显示方向。独立左转验证：去静态偏置陀螺积分 +57.737°，里程计 +59.810°，差约 2.073°，地图未冻结。

config/display_alignment.go2w_20261022.json 保留验证结果。API 保留源 XYZ，附加显示矩阵；前端对点云和轨迹应用同一变换，方向球显示前/左/上。实时帧原点仍为雷达，地图采用初始前向和重力参照。

前几次不足位移、符号冲突、时间回退冻结没有被认定通过，记录在 mapping_validation_2026-10-22.json。持续采样由操作员回复结束，避免短窗口漏掉动作；左转验证先估计静止陀螺偏置。

## 验收与边界

- 本地和服务器：46 项监控 Python 检查、184 项 VLA 回归通过；11 项 JavaScript 算法检查通过。
- 23 项合成浏览器检查通过，覆盖显示变换、清理、暂停、重连、地图、手机布局、无脚本/着色器错误。
- 最终真实 Chrome：实时 1675 显示点，聚合 16 帧/14505 点，世界地图 26781 点；实时和地图 display_direction_verified=true，mapping_version=2、fault=false，无浏览器错误。点数随场景变化。
- 自动下发运动命令为 0，现场动作全由用户遥控；motion_enabled=false，未开放自主导航。启动脚本通过 bash -n。

本轮是显示方向与定位显示链路验收。未用卷尺/角度计获得真值，因此米制尺度、精确转角和完整 base_link 安装平移/导航 TF 仍未验收，body_alignment_verified=false 保留。静态聚合只适合固定观察；世界地图有位姿补偿，不具备闭环重定位与导航精度承诺。更换安装或固件后须重验 profile。

真实场景截图只在本地 tmp/pcviz_live，未上传原始点云/视频或新增数据集。GitHub 包含聚合测试 JSON、标定矩阵和合成场景图片。原异常缓存备份仅保留服务器运行目录。

## 持久启动

运行目录 /mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor/。

1. 确认机器狗静止、网络和雷达在线。
2. 安装本机已验证 profile：cp config/display_alignment.go2w_20261022.json display_alignment.json；不要直接套用到其他机器。
3. 若 Point-LIO 已运行，先 bash /home/dell/go2w_vla/deploy/point_lio/stop_point_lio.sh。
4. bash run_mapping_stack.sh 启动适配器、持久参数文件的 Point-LIO 和同步地图，不依赖 /tmp 试验文件。地图进程重启后从新锚点重新生成。
5. Web 使用 /home/dell/go2w_vla/.venv/bin/python，由 go2wweb 提供。

VLA 默认参数及 start_point_lio.sh 已同步更新：使用校正话题时检查验证 profile，并按需启动适配器。完整机身外参配置保持未验证状态。

停止建图：先 stop_point_lio.sh，再 tmux kill-session -t mapworker 和 tmux kill-session -t imuadapter。独立 pcworker 仍可提供实时帧；地图显示历史状态。回滚保留旧备份，但不能宣称旧方向参数适用于移动建图。
