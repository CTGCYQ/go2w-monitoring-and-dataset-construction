# RealSense原始深度与工厂参数检查（2026-10-21）

用户确认机器狗已开机站立，雷达与深度相机均保持原装位置。软件运动门禁全程关闭，未发送姿态或运动指令。

## 实测结果

- 原装D435I、640×480、15FPS；30/30原始Z16帧和内存对齐帧通过，实测约14.78FPS。
- SDK读取meters_per_raw_unit=0.00100000005，raw=0表示无效，未硬编码毫米单位。
- 原生深度fx/fy约390.407；彩色fx/fy约606.463/606.344。两图尺寸相同并不意味着可共用内参。
- SDK对齐深度到彩色后，输出内参与彩色一致；彩色/深度时间戳同一时间域，最大差约0.044毫秒。
- 本轮场景有效像素约97.43%，中心5×5区域深度中位数约2.279–2.313米。此处没有用尺量作真值，因此不声称测距精度验收通过。
- 原始数值×单位与SDK get_distance差值最大约1.14e-7米。
- 工厂depth→color外参直接来自当前硬件，旋转输出明确为row-major，平移单位米；该变换不是camera→base_link。
- 不保存原始16位图、JPEG或轨迹；只保存工厂参数和聚合统计。

## 程序与复验

工具：dock/rs_probe.cpp；构建：bash dock/build_probe.sh，依赖现有librealsense2，不依赖OpenCV。

相机同一时间只能被一个采集程序占用。通过SSH连接扩展坞，在维护时执行：

```bash
cd /home/unitree/rs_dock
bash build_probe.sh
(
  set -e
  sudo systemctl stop rs_capture
  trap 'sudo systemctl start rs_capture' EXIT
  ./rs_probe 30
)
systemctl is-active rs_capture
```

自动化调用必须用finally/trap恢复rs_capture，探针失败也不能漏掉恢复。程序只读stream/factory calibration，没有调用相机写标定或硬件复位接口。

本轮采集服务已恢复active，两路深度相机网页图像各3/3完整解码成功，8000监控正常。

## 几何模块

VLA项目vla/depth_geometry.py使用config/realsense_factory.json，支持原生/已对齐Z16像素反投影，以及depth optical→color optical变换。光学轴为x向右、y向下、z向前。

当前工厂畸变系数全零；更换设备或分辨率后必须重新读取。若出现非零畸变，该简化反投影会明确拒绝，需使用SDK去畸变，不能静默采用针孔近似。

新增8项测试覆盖单位、无效值、不同内参、旋转矩阵存储顺序、畸变和不正确格式。最终本机发布副本113 passed（3.48s），80服务器113 passed（0.69s）；服务器几何模块编译检查通过。

## 未完成

- 网页现有伪彩色深度流未改为SDK空间对齐流；本轮对齐发生在探针内存中。
- 80服务器尚未接入连续原始深度通道，不能把可显示JPEG写成已具备在线深度避障。
- 相机/LiDAR到base_link外参、完整TF树和正式测距精度测试仍待完成。
- 板载已找到RealSense驱动与相机自身URDF，但未找到已验证的机身安装外参；不填猜测的单位变换。
