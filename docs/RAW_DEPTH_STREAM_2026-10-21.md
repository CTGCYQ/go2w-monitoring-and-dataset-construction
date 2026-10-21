# 连续原始深度通道与验收（2026-10-21）

## 实现与边界

扩展坞rs_capture继续采集RGB与原生深度并提供原有JPEG，同时用rs2::align将Z16深度对齐到彩色相机。新通道只保留最新帧在RAM中，不增加原始深度磁盘文件。80服务器通过现有SSH身份建立direct-tcpip通道，访问扩展坞127.0.0.1:8766；该端口不对机器人局域网监听。服务器DepthReceiver在内存中保留最新帧，请求周期最低0.2秒，传输开销使实际更新率低于相机15fps；不保证15fps交付。

新接口：
- HTTPS /api/depth/metric：在线状态、帧号、单位、光学帧、对齐内参、数据年龄与中心5×5有效像素中值（米）。中心全部为0时返回null，不表示0米。
- HTTPS /api/depth/raw：长度前缀JSON加原始Z16，小端、逐行紧密排列。离线/过期时503；Cache-Control:no-store。原有/depth/color与/depth/image仍为JPEG，其像素不能用于原始测距。

光学帧名称realsense_color_optical_frame仅声明数据轴向x右/y下/z前；本轮没有发布相机到机身的TF。已对齐数据使用彩色内参。Z16的0表示无效，米=raw×meters_per_unit；读取单位约0.0010000000475米。机身外参、标尺测距精度、持续避障仍未验收。

## 传输协议v1

客户端发送一个字节D，每次请求返回4字节网络字节序JSON长度、UTF-8 JSON、payload_bytes字节像素。JSON上限16384，像素上限1280×720×2；包含version、format=Z16_LE、alignment=color、width/height、frame_number、color_frame_number、meters_per_unit、fx/fy/ppx/ppy、畸变模型/系数、设备时间戳及时间域、captured_host_time_s和source_age_ms。

扩展坞墙上时间未校准，captured_host_time_s仅作来源时钟标记，禁止与80服务器UNIX时间直接相减。扩展坞用单调时钟计算source_age_ms；80用单调时钟记录请求开始和接收年龄，重复读取同一帧不刷新在线寿命。总年龄超过1.5秒拒绝返回原始帧；HTTP返回包还计入服务器缓存年龄。传输中断立即清除在线缓存并重连。客户端收到裸包后如需继续缓存，也必须自行计入本地停留时间。

## 验证

本机与80服务器各13项监控测试通过，包含旧点云门禁原有6项及深度通道7项。覆盖分片读取、截断、超大长度、非法元数据、单位换算、无效深度、接收端过期和HTTP年龄累计。扩展坞C++编译通过（原有system调用有返回值未处理告警）。

实测30次摘要请求全在线，6.749秒内23个不同帧号；最大来源年龄68.342毫秒、最大接收端年龄0.4001秒。原始帧640×480、614400字节、对齐到color。停止相机服务后摘要offline且原始接口503，finally重启后恢复online；RGB与伪彩JPEG各3次完整解码通过。具体统计见depth_stream_validation_2026-10-21.json。没有发送运动指令，运动门禁保持false，没有保存测试原始帧。

## 启动与回滚

新通道随rs_capture.service及监控server.py启动，无需新增独立常驻进程。80监控仍为https://192.168.1.80:8000。重启监控后自动连接相机；网络或相机不可用时接口保持offline并重试。

扩展坞备份：/home/unitree/rs_dock/rs_capture.bak_20261021_stream及rs_capture.cpp.bak_20261021_stream。80服务器server.py备份后缀.bak_20261021_depthstream。回滚时先停止对应服务，恢复备份，再启动并核对状态。回滚旧采集二进制会让新增原始深度接口离线，原有JPEG功能保留。新增源码头文件为dock/raw_depth_stream.hpp，与rs_capture.cpp一同编译；仍链接librealsense2、OpenCV和pthread。
