# application-core 改动跟踪记录

本文档持续记录 `/home/yunle/application-core` 从原始仓库克隆后，为 Yunle 无人小车适配而实施的工程改动、设备侧配置和验证结果。

> 维护约定：以后每次对 `application-core` 实施修改，都应在同一次任务中更新本文档。工程源码、Profile 配置、设备持久配置和仅用于测试的临时操作必须分开记录；尚未实施的方案不能写成已完成。

## 1. 原始基线

| 项目 | 内容 |
| --- | --- |
| Git 仓库 | `https://github.com/bliu37/application-core.git` |
| 分支 | `main` |
| 克隆后的基线提交 | `d0b762ca9740794bcbc689f0a49596b4f4ac654d` |
| 基线提交说明 | `fea: upgrade to 11.0.0` |
| Apollo 版本 | Apollo 11.0，包管理安装方式 |
| 当前适配 Profile | `profiles/yunle` |

本记录只说明基线之后的本地适配，不代表这些修改已经提交或推送到 Git 远端。截至 2026-07-31，适配文件仍处于本地未提交状态。

## 2. 当前适配范围和状态

| 子系统 | 硬件/用途 | 当前状态 |
| --- | --- | --- |
| 激光雷达 | 镭神 LS-C16 V4 | 驱动已适配，Scan 和 PointCloud2 均已验证有数据 |
| Dreamview Plus | 传感器预览/控制联调模式 | 已增加 Yunle 模式及传感器、定位、底盘和 Control 独立开关；临时时间桥保留并明确命名为 `PointCloudBridge` |
| GNSS/INS | 华测 CGI-230，COM1，460800 baud | Apollo 内置 `HUACE_TEXT` 已打通；IMU、定位和 RTK 固定解已验证 |
| 底盘/CAN | Yunle 底盘、eth_can | receiver 和受限控制链路已迁移；1.5 km/h 纵向及左右曲率轨迹已实车验证 |
| 室内激光 SLAM | LS-C16 V4 + CGI-230 IMU | 已增加 Dreamview 室内建图模式；LIORF 建图链路已完成静止、直行、转角和地图保存验证 |
| 室内点云定位 | Map03 点云地图 + PCL-OMP NDT | 已增加不修改 Apollo 原生定位的 Yunle PCL-OMP NDT 入口；静止 60 秒诊断显示定位不再明显漂移 |
| 室内导航 | Map03 HD Map、Routing、Planning、Control | 已固化 Dreamview 室内导航模式；地图、Routing、Planning 轨迹、START gate 和低速实车导航已进入当前稳定版 |

## 3. 2026-07-31：建立 Yunle Profile

新增并启用：

```text
profiles/yunle/
profiles/current -> /apollo_workspace/profiles/yunle
```

用途：把 Yunle 小车的硬件参数和 Dreamview 模式放在 Profile 中，避免直接修改 AEM 安装目录 `.aem/envroot/opt/apollo/neo/src` 下的 Apollo 发行包源码。

`profiles/current` 是执行 `aem profile use yunle` 后形成的当前 Profile 链接，属于本机工作区状态。

## 4. 2026-07-31：LS-C16 V4 激光雷达适配

### 4.1 新增雷达 Profile 配置

新增文件：

```text
profiles/yunle/modules/drivers/lidar/lslidar/conf/lslidar16v4_conf.pb.txt
profiles/yunle/modules/drivers/lidar/lslidar/dag/lslidar.dag
```

主要配置：

```text
型号：LSLIDAR_C16_V4
雷达 IP：192.168.1.200
主机 IP：192.168.1.102（主机网络配置，不写入该 pb.txt）
MSOP 端口：2368
DIFOP 端口：2369
转速：600 rpm
点云 frame_id：lslidar16v4
```

发布通道：

```text
/apollo/sensor/lslidar16v4/Scan
/apollo/sensor/lslidar16v4/PointCloud2
```

### 4.2 新增最小修复组件

新增源码包：

```text
modules/drivers/lidar/lslidar_c16v4_custom/
├── BUILD
├── cyberfile.xml
└── lslidar_component.cpp
```

原因：Apollo 11 包内原有 LS lidar 组件在 Scan 队列暂时为空、`popWait()` 返回失败时，会结束点云转换线程。结果是 `/Scan` 持续有数据，但 `/PointCloud2` 没有输出。

修复方式：保留 Apollo 原有驱动和解析器，只替换组件层的队列轮询行为。在 Cyber 未关闭时持续轮询；队列暂空时等待 1 ms 后重试，不再永久退出转换线程。

DAG 改为加载：

```text
modules/drivers/lidar/lslidar_c16v4_custom/liblslidar_c16v4_custom_component.so
```

没有修改 `.aem/envroot` 中的 Apollo 发行包源码，也没有复制或重写 LS-C16 V4 底层协议解析器。

### 4.3 验证结果

```text
/apollo/sensor/lslidar16v4/Scan：有数据
/apollo/sensor/lslidar16v4/PointCloud2：约 10 Hz
```

相关本地包已成功编译并安装为：

```text
/opt/apollo/neo/lib/modules/drivers/lidar/lslidar_c16v4_custom/liblslidar_c16v4_custom_component.so
```

## 5. 2026-07-31：Dreamview Plus 点云预览

### 5.1 新增临时定位时间桥

新增源码包：

```text
modules/tools/yunle_dreamview_bridge/
├── BUILD
├── cyberfile.xml
├── localization_time_bridge.cc
├── dag/localization_time_bridge.dag
└── launch/localization_time_bridge.launch
```

功能：以 20 ms 定时周期（约 50 Hz）向 `/apollo/localization/pose` 发布带当前时间戳、单位四元数的最小 `LocalizationEstimate`，使 Dreamview Plus 在正式定位尚未启动时可以预览实时点云。

重要限制：这是仅供传感器预览使用的占位定位。启动正式 localization 前必须关闭 `PointCloudPreview`，避免两个发布者同时写 `/apollo/localization/pose`，更不能把占位消息用于自动驾驶控制。

该包已成功编译并安装为：

```text
/opt/apollo/neo/lib/modules/tools/yunle_dreamview_bridge/libyunle_dreamview_bridge_component.so
```

### 5.2 新增 Dreamview 模式

新增文件：

```text
profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_sensor_preview.pb.txt
```

初始 Dreamview Plus 模式名称：`Yunle Sensor Preview`。

初始 Modules 开关：

```text
LslidarC16V4
PointCloudPreview
Gnss
```

底盘包尚未迁移，因此当前模式没有加入未完成的 Yunle Canbus 模块。
后续底盘接入和最终模式拆分见第 9～11 节。

## 6. 2026-07-31：华测 CGI-230 GNSS/INS 接入

### 6.1 硬件识别结论

CGI-230 使用：

```text
接口：RS232-A / COM1
Linux 设备：/dev/ttyUSB0
USB 转串口：Prolific Technology USB-Serial Controller
稳定设备路径：/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_DWCRm153609-if00-port0
波特率：460800
```

`/dev/ttyUSB1`（`/dev/wheeltec_FDI_IMU_GNSS`）是另一套 WheelTec FDI IMU/GNSS，不用于 CGI-230。

### 6.2 新增 GNSS Profile 配置

新增文件：

```text
profiles/yunle/modules/drivers/gnss/conf/gnss_conf.pb.txt
```

主要配置：

```text
format: HUACE_TEXT
serial device: Prolific 的 /dev/serial/by-id 路径
baud_rate: 460800
rtk_solution_type: RTK_RECEIVER_SOLUTION
imu_type: CPT_XW5651
投影：WGS84 / UTM zone 50
use_gnss_time: true
```

Apollo 11 内置 `HUACE_TEXT` 解析 `$GPCHC` 文本，不解析接收机原先输出的 `HCINSPVATZCB` 二进制。因此 Profile 登录命令只调整 COM1 的当前运行态输出：

```text
unlog com1 HCINSPVATZCB
unlog com1 GPCHC
log com1 GPCHC ontime 0.01
```

这里特意不使用 `unlogall`，避免清除 COM2、CAN、以太网等其他接口的输出配置；Profile 中也不包含 `saveconfig`，所以 Apollo 启动不会自行修改接收机的持久配置。

### 6.3 设备侧临时测试

已在 COM1 临时验证以下切换流程：

1. 临时恢复 `HCINSPVATZCB`，确认 `aa cc 48 43` 特征的二进制数据正常输出。
2. 仅停止 COM1 上的 `HCINSPVATZCB` 和已有 `GPCHC`。
3. 临时开启 `$GPCHC`，周期 0.01 秒（100 Hz）。
4. 连续读取确认输出是纯 `$GPCHC` 文本，没有二进制混杂。

实测 `$GPCHC` 每帧 GPS 时间增加 0.01 秒，卫星数约 27～30，状态字段为 42；定位结果达到组合导航 RTK 固定状态。

设备侧 `saveconfig` 的执行结果和断电重启后的保持情况目前尚未记录为已验证。完成断电重启验证前，应把设备持久化状态视为“待确认”。

### 6.4 Apollo 通道验证结果

```text
/apollo/sensor/gnss/raw_data：持续有数据
/apollo/sensor/gnss/imu：约 100 Hz
/apollo/sensor/gnss/corrected_imu：约 100 Hz
/apollo/sensor/gnss/best_pose：约 1.1 Hz
```

`best_pose` 实测：

```text
sol_status: SOL_COMPUTED
sol_type: NARROW_INT
latitude: 约 40.0639266
longitude: 约 116.36232
height_msl: 约 33.4～33.8 m
```

说明：CGI-230 的 `$GPCHC` 为 100 Hz，不代表 Apollo 的 `best_pose` 也是 100 Hz。Apollo 11 华测文本解析器对 `best_pose` 有约 1 Hz 的发布节流；当前决定保留默认行为，高频数据使用约 100 Hz 的 IMU、corrected_imu 和 odometry。没有修改 Apollo GNSS 解析器源码，也没有实施 `best_pose` 20 Hz 改造。

已知限制：Apollo 11 内置华测文本解析器没有把 `$GPCHC` 的 `NSV1/NSV2` 填入 `GnssBestPose` 的卫星数字段，因此消息中的 `num_sats_*` 可能显示 0，这不等于接收机没有跟踪卫星。

## 7. 工作区和构建相关状态

根目录 `WORKSPACE` 从空文件变为 Apollo Bazel 工作区依赖定义。这是 AEM/buildtool 初始化或编译工作区时生成的构建环境文件，不属于 Yunle 传感器业务逻辑；当前它仍表现为相对 Git 基线的本地修改。

曾遇到并已处理的构建问题：

- 新雷达包的 `cpplint` load 标签最初写法不适配当前工作区，已改为 `load("//tools:cpplint.bzl", "cpplint")`。
- Dreamview 模式曾因引用尚不存在的 `canbus-vehicle-yunle` 包导致 PackageAttrErr；最终模式不再声明这个未迁移包，之后全工作区编译成功。

以下内容不是应提交的功能源码：

```text
dumps/          # 运行日志/转储
helpdocs.zip    # 本地资料压缩包
profiles/current # 本机当前 Profile 链接，是否入库应单独决定
```

## 8. 尚未实施的适配与待办

- 标定 CGI-230 的 `ant2bodyoffset`、`ins2antoffset`、`insangle`、`headingoffset`、`wheeltread`、`ant2outposoffset` 等安装参数；这些参数会影响正式定位精度。
- 确认 CGI-230 执行 `saveconfig` 后，断电重启仍直接输出纯 `$GPCHC` 100 Hz。
- 在提交前制定 `.gitignore` 和提交范围，排除 `dumps/`、压缩包及其他运行产物。
- 使用正式 RTK Localization、Planning 或 Control 前，确认临时
  `PointCloudBridge` 已关闭，避免两个发布者同时写 `/apollo/localization/pose`。

## 9. 2026-08-01：Dreamview 实车联调开关

- 目的：在 `Yunle Sensor Preview` 中通过按钮启动静态 TF、GNSS、RTK
  Localization、底盘 receiver 和 Apollo Control，减少终端手工启动步骤。
- 文件：修改
  `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_sensor_preview.pb.txt`。
- 内容：增加 `StaticTf`、`RtkLocalization`、`Control` 和
  `YunleApolloControlTest`；保留接收模式 `YunleChassisPreview`；GNSS 开关沿用
  原配置。`YunleChassisPreview` 与 `YunleApolloControlTest` 共用 UDP 端口
  `8002`，运行时必须二选一。
- 将临时定位按钮改名为 `PointCloudBridge`，明确它只供没有正式定位时的点云
  预览使用；正式 RTK Localization、Planning 或 Control 运行时必须关闭。
- 验证：HMI mode 文本已通过 `protoc --encode=apollo.dreamview.HMIMode`
  解析校验；各按钮已在 Dreamview 中成功启动对应 Cyber 进程。
- 影响/回退：需要无 RTK 的临时点云预览时可打开 `PointCloudBridge`，使用正式
  RTK Localization、Planning 或 Control 时必须关闭。改名不改变 DAG 和进程组。
- 状态：已完成并验证。

## 10. 2026-08-02：Dreamview 模式更名

- 目的：使模式名称覆盖当前的传感器、定位、底盘和实车控制测试用途。
- 文件：将
  `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_sensor_preview.pb.txt`
  更名为
  `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_vehicle_tests.pb.txt`。
- 内容：Dreamview 显示名称由文件名自动转换，因此模式由
  `Yunle Sensor Preview` 变为 `Yunle Vehicle Tests`；内部模块配置未改变。
- 验证：新文件已通过 `protoc --encode=apollo.dreamview.HMIMode` 解析校验；
  Dreamview 已成功加载并切换到 `Yunle Vehicle Tests`，旧模式已清理。
- 影响/回退：重新应用 Yunle Profile 后，模式列表只保留
  `Yunle Vehicle Tests`；回退时恢复旧文件名并先切换 KVDB 当前模式。
- 状态：已完成。

## 11. 2026-08-02：拆分传感器测试与车辆控制测试模式

- 目的：把纯传感器/底盘预览与具备实车下发能力的控制联调入口分开，减少模式内
  的互斥开关和误操作风险。
- 文件：新增
  `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_sensor_tests.pb.txt`；
  修改
  `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_vehicle_tests.pb.txt`。
- 内容：`Yunle Sensor Tests` 包含 `LslidarC16V4`、`PointCloudBridge` 和
  只接收反馈的 `YunleChassisPreview`。`Yunle Vehicle Tests` 保留
  `LslidarC16V4`，移除 `PointCloudBridge` 与 `YunleChassisPreview`，继续提供
  `StaticTf`、`Gnss`、`RtkLocalization`、`YunleApolloControlTest` 和 `Control`。
- 验证：两份 HMI mode 配置均通过
  `protoc --encode=apollo.dreamview.HMIMode` 解析校验。
- 影响/回退：新增模式文件需要重新应用 Yunle Profile 并重启 Dreamview Plus；
  两个模式仍不得在后台同时保留占用相同进程组或 UDP 端口的模块。
- 状态：Profile 已重新应用，两个模式均已在 Dreamview 中验证可用。

## 12. 2026-08-02：Yunle JD03 底盘反馈组件迁移

- 目的：在 Apollo 11 包管理工作区中接收 Yunle JD03 V4.5 底盘反馈，并用标准
  Apollo `Chassis` 和独立详情消息提供车辆状态。
- 文件：新增 `modules/canbus_vehicle/yunle/` 源码包、BUILD/cyberfile、协议解析、
  protobuf、单元测试与中文说明；新增
  `profiles/yunle/modules/canbus_vehicle/yunle/` 下的接收配置、DAG 和 launch。
- 内容：完成 ETH-CAN 13 字节封装接收及 JD03 反馈解析，覆盖通信、挡位、驻车、
  驾驶模式、车速、四轮转速、前后转角、目标速度、制动、BMS 和告警。轮胎半径
  使用 `0.133 m`；实车未提供有效前轮 RPM，因此标准 Chassis 将前轮轮速标记为
  无效，详情话题仍保留原始值。
- 通道：发布 `/apollo/canbus/chassis` 和
  `/apollo/canbus/yunle_chassis_detail`。独立详情 protobuf 避免与 Apollo 其他车型
  的私有 `chassis_detail` 类型冲突。
- 安全默认：正常 `yunle_chassis_receiver.launch` 明确设置
  `enable_control_send: false`，不创建控制 Reader、不发送控制帧，并将驾驶模式
  报告为人工模式，适合 Dreamview 底盘预览和遥控器诊断。
- 验证：实车通信、挡位、驻车、速度、转角、后轮 RPM、制动和目标速度反馈均已
  在详情通道验证；普通 receiver 启动后保持只接收状态，遥控器仍可正常控制。
- 状态：已完成并通过实车反馈验证。

## 13. 2026-08-02：RTK Localization 与 Yunle Control Profile

- 目的：为 Apollo Control 提供连续的正式定位输入和适配小车低速特性的横纵向
  控制配置。
- 文件：新增
  `profiles/yunle/modules/localization/conf/rtk_localization.pb.txt`、
  `profiles/yunle/modules/transform/conf/static_transform_conf.pb.txt`，以及
  `profiles/yunle/modules/control/` 下的 Control flag、pipeline、LQR 横向和 PID
  纵向配置。
- Localization：使用 `/apollo/sensor/gnss/odometry`、`corrected_imu` 和
  `ins_stat` 生成 `/apollo/localization/pose`；输出包含有限有效的位置、姿态、
  线速度、线加速度、角速度和欧拉角。当前控制链路使用
  `imu_frame_id: "localization"`，不依赖额外静态 TF 才能启动。
- Control：pipeline 使用 `LatController` 和 `LonController`；控制周期为 10 ms，
  纵向控制器输入上限设为 `1.5 km/h`，并配置适用于低速小车的 LQR 增益、预瞄和
  PID 参数。
- 验证：RTK Localization 已稳定发布完整定位消息；Apollo Control 能基于该定位、
  Yunle Chassis 和测试 Planning 轨迹持续产生有限有效的速度、制动和转向命令。
- 状态：已完成并用于后续实车全链路测试。

## 14. 2026-08-02：受限低速控制通路与安全保护

- 目的：在正常只接收模式之外，提供明确隔离、边界固定、可观测的实车调试控制
  入口。
- 文件：新增 `yunle_control_protocol` 0x121 编码器及离线/单元测试，扩展
  `YunleChassisReceiverComponent` 的控制接收、联锁、下发和诊断；新增
  `yunle_chassis_control_test` 与 `yunle_chassis_apollo_control_test` 两套测试
  Profile/DAG/launch，以及 `tools/yunle_control_test_publisher.py`。
- 隔离：直接底盘测试订阅 `/apollo/canbus/yunle_test_control`；Apollo 链路测试
  订阅 `/apollo/control`。正常 receiver 和两套测试 receiver 共用 UDP 端口
  `8002`，运行时只能选择一个。
- 固定边界：命令速度硬上限 `1.5 km/h`，实测速度阻断阈值 `1.7 km/h`，转向硬
  上限 `90%`，命令超时 200 ms，失效制动最多发送 500 ms。SCU 的 0.1 km/h
  分辨率采用不超过请求值的向下量化。
- 联锁：校验通信和核心反馈新鲜度、自动驾驶物理开关、挡位、驻车、告警、硬件
  遥控刹车/急停、ControlCommand 状态及 engage advice，并拒绝 NaN/Inf 等非法
  数值。命令失效后发送零速、回正、制动，再停止发送；静止无有效命令时不会持续
  反复抢占制动。
- 测试工具：支持受限的 N→D、零速制动换挡、释放制动、运动和停车分阶段流程，
  结束时发布零速制动，并输出 SCU 目标、实际车速、转角、联锁、failsafe 和发送
  计数。一次性 Python Cyber 进程能够可靠退出。
- 验证：直行、正负转向、换挡、停车以及硬件制动联锁均已实车验证；JD03 四轮
  转向协议的前后通道按同一命令符号编码，实车机械表现为前后轮反向转动。最大
  `90%` 转向边界和 `1.5 km/h` 速度边界均已验证可用。
- 状态：已完成并保留正常模式默认禁止发送的安全边界。

## 15. 2026-08-02：Planning → Control → Yunle 全链路实车验证

- 目的：不直接向 `/apollo/control` 写转向值，由 Apollo Control 根据 Planning
  轨迹计算纵向和横向控制，验证完整控制链路。
- 文件：新增 `tools/yunle_planning_test_publisher.py`；Apollo 链路测试 receiver
  使用 `report_auto_driving_mode: true`，仅在底盘反馈确实处于自动驾驶模式时向
  Control 报告 `COMPLETE_AUTO_DRIVE`。
- 轨迹工具：只发布 `/apollo/planning`。支持最高 `1.5 km/h` 的直线轨迹，以及
  `--turn left/right` 恒定曲率圆弧；圆弧半径限制为 `[10, 100] m`，默认
  `10 m`。每次轨迹以最新定位姿态为起点，结束时发布零速直线轨迹。
- 纵向结果：`1.0 km/h` 直线请求时，receiver 接收约 `1.01 km/h` 的 Control
  计算值并正常下发，SCU 目标反馈约 `0.9 km/h`；`1.5 km/h` 请求被严格钳制为
  `1.500 km/h`，SCU 目标反馈 `1.4 km/h`。两次运动和停车阶段均无 receiver
  failsafe 或发送错误，车辆表现正常。
- 横向结果：在 `1.0 km/h`、半径 `10 m` 条件下，左转轨迹曲率 `+0.1 1/m`，
  Apollo 输出约 `+26%` 转向；右转曲率 `-0.1 1/m`，输出约 `-25%` 转向。左右
  响应基本对称，车辆实际转向方向正确。
- 结论：已实车打通
  `/apollo/planning → Apollo Control → /apollo/control → Yunle receiver → SCU`
  的纵向与横向控制链路。
- 状态：受限低速直线和左右曲率测试已完成并通过实车验证。

## 16. 2026-08-02：LS-C16 V4 适配包硬件化命名与车型清单

- 目的：将雷达组件按硬件型号而不是当前车辆品牌管理，便于后续不同车型复用；
  同时为 Yunle JD03 建立集中列出底盘、传感器、Profile 和安全边界的车型清单。
- 文件：将 `modules/drivers/lidar/lslidar_yunle/` 重命名为
  `modules/drivers/lidar/lslidar_c16v4_custom/`；同步修改 BUILD target、
  cyberfile 包名和
  `profiles/yunle/modules/drivers/lidar/lslidar/dag/lslidar.dag` 动态库路径；新增
  `profiles/yunle/README.md`。
- 内容：动态库改名为 `liblslidar_c16v4_custom_component.so`，模块标识改为
  `lslidar_c16v4_custom`。雷达参数、`LslidarComponent` 类、Scan/PointCloud2
  通道以及队列轮询修复行为均未改变。
- 验证：已完成旧名称全工作区引用检查，源码、BUILD、cyberfile、Profile DAG
  和文档均只引用新名称；容器内重新编译、Profile 应用和点云在线验证待执行。
- 影响/回退：新 DAG 依赖新路径下编译安装的动态库；编译前 Dreamview 雷达按钮
  无法使用新路径。回退时需同时恢复源码目录、BUILD/cyberfile 名称和 DAG 路径。
- 状态：源码和 Profile 重命名已完成，运行验证待完成。

## 17. 2026-08-03：室内 LIORF 建图链路与测试 TF 固化

- 目的：在没有 GNSS 室内位置的环境中，使用 LS-C16 V4 点云和 CGI-230 IMU 完成
  室内激光 SLAM 建图，并把建图链路做成 Dreamview 可启动模式。
- 文件：新增 `Yunle Indoor Mapping Tests` HMI 配置；新增
  `profiles/yunle/modules/drivers/gnss/conf/yunle_indoor_gnss_conf.pb.txt`、
  `profiles/yunle/modules/drivers/gnss/dag/yunle_indoor_gnss.dag`、
  `profiles/yunle/modules/transform/` 下的室内测试 TF 配置、DAG 和外参参数；
  新增 `profiles/yunle/modules/loam_velodyne_indoor/` 建图配置；扩展
  `modules/tools/yunle_dreamview_bridge` 的室内 TF、地图保存和诊断工具。
- 内容：室内 GNSS 入口只保留 IMU 数据，不再发布室外 GNSS 位置生成的
  `world -> imu` 动态 TF；`IndoorTestTf` 周期发布
  `localization -> imu -> lslidar16v4`，使用当前尺量初值
  `(0.03, -0.33, 0.67) m, -90 deg`；建图模式只包含传感器、测试 TF、LIORF
  mapping 和只读底盘 receiver，不启动 Planning 或 Control。
- 验证：已完成静止漂移、直行距离、左右转角和地图保存验证；生成的点云地图作为
  后续 Map03 定位和 HD Map 制作输入。
- 影响/回退：该链路不生成 Apollo 语义 HD Map；启动正式定位或导航前必须关闭
  建图入口，避免多个进程同时写定位相关话题或 TF。
- 状态：已完成并通过室内建图链路验证。

## 18. 2026-08-03 至 2026-08-04：Apollo CPU-NDT 固定地图定位 A/B

- 目的：把 LIORF 相对里程计和固定点云地图接入 Apollo NDT 定位，定位并隔离
  室内短地图中米级漂移、坐标精度、轴向和外参问题。
- 文件：新增和修改
  `modules/tools/yunle_dreamview_bridge/indoor_ndt_input_bridge.cc`、
  `indoor_ndt_pose_stabilizer.cc`、`yunle_indoor_ndt_map_prepare.py`、
  `yunle_indoor_ndt_localization_diagnostic.py`、
  `yunle_indoor_ndt_alignment_diagnostic.py` 及相关 DAG；新增
  `profiles/yunle/modules/localization/conf/`、`dag/` 和 `msf/params/` 下的室内
  NDT 配置；新增 `Yunle Indoor Localization Tests` HMI 模式。
- 内容：依次保留基线 NDT、float-safe 原点、`-90 deg` 点云轴向、Map03 地图、
  尺量外参和 Z-stable 等 A/B 入口。float-safe 入口把地图坐标原点从
  `(10000, 9000000, 0)` 改为 `(10000, 10000, 0)`，避免 Apollo CPU-NDT 在
  900 万米级 Y 坐标附近使用 float 计算时出现米级量化误差；Z-stable 入口发布
  raw topics 后保留 NDT X/Y，并使用 LIORF/bridge 的 Z、roll、pitch 生成最终
  `/apollo/localization/pose`。
- 验证：基线和中间 A/B 入口能够发布 NDT 位姿，但 CPU-NDT 在当前室内短地图上
  仍出现静止漂移或高度局部解；这些入口保留为诊断和回归对照，不作为当前室内
  导航默认定位。
- 影响/回退：所有 NDT 入口发布相同最终定位话题，任一时刻只能启动一个；继续
  调定位时必须用 60 秒 `planar_delta`、span 和 raw/final 对比诊断，而不能只看
  话题是否活跃。
- 状态：A/B 诊断链路已完成；最终导航默认入口已转向第 19 节的 PCL-OMP NDT。

## 19. 2026-08-05：引入 Yunle 自有 PCL-OMP NDT 定位入口

- 目的：参考 `/home/yunle/art107/src/navigation/ndt_localizer` 和
  `/home/yunle/art107/src/navigation/ndt_omp` 中已在 ROS 下验证过的定位实现，在
  Yunle 目录内实现可替换 Apollo 原生 CPU-NDT 的固定地图定位组件，同时保持
  Apollo 话题链路不变。
- 文件：新增
  `modules/tools/yunle_dreamview_bridge/indoor_pclomp_ndt_localizer.cc`，引入
  `modules/tools/yunle_dreamview_bridge/third_party/ndt_omp/` 下的 `pclomp`
  源码和头文件，扩展 `BUILD` 增加
  `libyunle_indoor_pclomp_ndt_localization_component.so`；新增对应 DAG 和 Profile
  tools DAG。
- 内容：PCL-OMP localizer 直接加载
  `jd03_indoor_20260803_03/global.pcd`，在内存中加 `(10000, 10000, 0)` 偏移后
  做整图 NDT 匹配；继续复用 LIORF input bridge 和 Z-stable stabilizer，raw
  输出仍走 `/apollo/yunle/indoor/ndt/raw_pose` 与
  `/apollo/yunle/indoor/ndt/raw_lidar`，最终定位仍发布到
  `/apollo/localization/pose` 和 `/apollo/localization/ndt_lidar`。
- 验证：Dreamview 和 Cyber Monitor 观察定位不再明显漂移；60 秒定位诊断得到
  `paired_samples=192`、`offset_delta_planar=0.0015m`、`max_offset_planar=0.4577m`、
  `z_raw_minus_final_span=0.0105m`，raw 与最终 X/Y 对齐，raw Z 相对最终约
  `-0.36m`。当时诊断工具未在窗口内匹配到 PCL-OMP 日志样本，后续如需调参仍应
  继续观察 `trans_prob` 和 accepted 状态。
- 影响/回退：不修改 Apollo 原生 CPU-NDT；回退时切回第 18 节的任一 NDT A/B
  入口即可。PCL-OMP 和 CPU-NDT 入口不能同时启动。
- 状态：已作为当前室内导航默认定位入口使用。

## 20. 2026-08-05：Map03 室内 HD Map、Routing 和 Planning 固化

- 目的：把已稳定的室内点云定位坐标系转成 Apollo 可用的单车道 HD Map，并打通
  Dreamview Routing、External Command、Planning 和空 Prediction 输入。
- 文件：新增 `modules/tools/yunle_dreamview_bridge/yunle_indoor_hdmap_generate.py`、
  `indoor_planning_command_adapter.cc`、`indoor_planning_heading_adapter.cc`、
  `indoor_empty_prediction_publisher.cc` 及对应 DAG；新增
  `profiles/yunle/modules/map/data/yunle_indoor_map03_planning/` 下的
  `base_map`、`routing_map`、`sim_map` 和测试 waypoint 文件；新增
  `profiles/yunle/modules/routing/`、`profiles/yunle/modules/planning/`、
  `profiles/yunle/modules/external_command/` 相关配置。
- 内容：`yunle_indoor_map03_planning` 使用稳定 PCL-OMP NDT 坐标系下的原始
  LIORF 拟合中心线，不再保留旧 NDT 验证阶段为补偿定位偏移而加入的额外中心线
  平移；车道宽度保持 `3.0 m`。`IndoorCommand` 启动
  `external_command_process` 和 `old_routing_adapter`，将 Dreamview 的
  `/apollo/routing_request` 转成 Planning 使用的 lane-follow command。
  `IndoorPlanningHeadingAdapter` 在 route 可用后，把 Planning 私有 localization
  topic 的 heading 对齐到当前 route 方向，解决稳定定位 raw/final 坐标约定下
  车头朝向与车道方向不一致的问题。
- 参数：室内 Planning 固定 `planning_upper_speed_limit=0.500 m/s`、
  `default_cruise_speed=0.500 m/s`，并设置 `destination_check_distance=20.0 m`
  以便短路线一开始就生成 destination stop fence。
- 验证：Dreamview 可加载 `yunle_indoor_map03_planning`；点击 `Send Routing`
  后已验证 Routing 有响应、红色 route 和 Planning 轨迹正常显示，车头方向修正后
  与车道方向一致。
- 影响/回退：`Save Editing` 只是保存 route 编辑，实际发送 route 仍以
  Dreamview 的 `Send Routing` 为准；地图、定位和 Planning 必须使用同一 Map03
  坐标系。
- 状态：已完成并作为当前室内导航默认地图和 Planning 链路。

## 21. 2026-08-05：Dreamview 室内导航模式固化

- 目的：把已经验证的室内定位、Routing、Planning 和受控底盘链路固化成
  Dreamview Plus 可启动的完整室内导航模式。
- 文件：新增
  `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_indoor_navigation.pb.txt`，
  同步更新 `profiles/yunle/README.md` 和 Profile tools DAG。
- 内容：`Yunle Indoor Navigation` 包含 `LslidarC16V4`、`GnssImu`、
  `IndoorTestTf`、`IndoorLiorfOdometry`、
  `IndoorPclOmpNdtLocalizationMap03MeasuredZStable`、
  `IndoorPlanningHeadingAdapter`、`IndoorRouting`、`IndoorCommand`、
  `IndoorEmptyPrediction`、`IndoorPlanning`、`YunleApolloControlTest` 和
  `Control`。默认 operation 保持 `Record`，需要实车控制时再显式切到
  `Waypoint_Follow` 或 `Auto_Drive` 并按 Dreamview 底部 `Start`。
- 验证：模式已能在 Dreamview 中加载地图、发送 Routing、产生 Planning 轨迹并
  进入低速实车导航。导航模式中只保留 PCL-OMP NDT 定位入口，避免多余 NDT 按钮
  在完整导航模式下造成重复定位发布者。
- 影响/回退：该模式会启动发送使能的 `YunleApolloControlTest`，不能和
  `YunleChassisPreview` 或任何其他占用 UDP `8002` 的 receiver 同时运行；只做
  定位测试时应使用 `Yunle Indoor Localization Tests`。
- 状态：当前稳定版。

## 22. 2026-08-05：小车车辆参数、START gate 和终点保护

- 目的：适配小鱼 800 的真实车身尺寸，避免 Apollo 默认 Lincoln MKZ 车身模型
  影响短距离室内路线；同时阻止发送 route 后未按 Dreamview `Start` 就自动起步。
- 文件：新增 `profiles/yunle/modules/common/data/vehicle_param.pb.txt`；修改
  `modules/canbus_vehicle/yunle/yunle_chassis_receiver_component.cc`、
  `modules/canbus_vehicle/yunle/proto/yunle_chassis_receiver.proto`、
  `profiles/yunle/modules/canbus_vehicle/yunle/conf/yunle_chassis_apollo_control_test_conf.pb.txt`
  和相关 README；新增/扩展 `yunle_chassis_diagnostic.py` 与
  `yunle_indoor_navigation_snapshot.py`。
- 内容：车辆参数改为车长 `0.900 m`、车宽 `0.700 m`，IMU 到车头 `0.550 m`、
  到车尾 `0.350 m`、到左边 `0.380 m`、到右边 `0.320 m`。
  `require_control_pad_start: true` 要求 ControlCommand 中出现 Pad `START` 后才向
  SCU 转发 `0x121` 控制帧，避免只发送 Routing/Planning 轨迹就启动车辆。
  `release_to_remote_on_terminal_stop: true` 保留终点释放保护，但当前稳定版要求
  Planning 报告 destination/reference-end/mission-complete、车辆静止且
  `ControlCommand` 为零速后，才等待 800 ms 并发送 500 ms 的 remote/neutral
  释放帧。
- 验证：已验证发送 route 后不会绕过 Dreamview `Start` 直接起步；短路线能够在
  START 后产生控制并行驶到 Planning 停止位置。最近两次导致 Start 失效或终点
  释放计时异常的修改已回退，当前保留较稳定的零速 ControlCommand 门槛版本。
- 影响/回退：修改 C++ 和 proto 后必须重新 `buildtool build`、应用 Yunle Profile
  并重启相关模块；终点后是否能完全释放遥控权仍应继续用
  `terminal_stop_released_to_remote`、`remote_release_frame_count` 和实车遥控响应
  做闭环确认。
- 状态：START gate 和低速导航为当前稳定版；终点释放权交接继续作为后续实车
  验证项。

## 23. 2026-08-05：室内导航预检和诊断工具

- 目的：把室内导航链路的关键话题、地图、Routing、Planning、Control 和底盘状态
  做成可重复运行的命令行诊断，避免只凭 Dreamview 画面判断链路是否正常。
- 文件：新增 `modules/tools/yunle_dreamview_bridge/` 下的
  `yunle_indoor_navigation_readonly_preflight.py`、
  `yunle_indoor_routing_preflight.py`、`yunle_indoor_planning_preflight.py`、
  `yunle_indoor_control_zero_speed_preflight.py`、
  `yunle_indoor_navigation_snapshot.py`、`yunle_indoor_odometry_diagnostic.py`、
  `yunle_tf_diagnostic.py`、`yunle_slam_status_compatibility_diagnostic.py` 等工具；
  扩展 `modules/canbus_vehicle/yunle/tools/` 下的控制和底盘诊断工具。
- 内容：预检工具分别覆盖只读启动、Routing 响应、Planning 轨迹、0 速度 Control
  链路和导航快照；快照工具汇总 localization、planning localization、
  routing_request/response、planning_command、planning、control、chassis 和
  Yunle detail，便于定位“没有 route”“Planning not ready”“Control 未按 START
  gate 放行”“终点状态未释放”等问题。
- 验证：这些工具已用于确认 Dreamview 模式能加载、地图为
  `yunle_indoor_map03_planning`、Routing 有响应、Planning 有轨迹、0 速度 Control
  链路安全，以及后续排查 START gate、停止墙和终点控制权问题。
- 影响/回退：诊断脚本只读或受限发布测试消息，不应与实车控制入口混用；运行中
  产生的 `*.INFO`、`*.log.INFO.*`、`__pycache__/` 等文件属于运行产物，不应纳入
  功能提交。
- 状态：已完成并用于当前室内导航联调。

## 24. 后续记录格式

后续每次经确认实施改动时，在本文档顶部当前状态中同步更新，并在本节之前追加如下条目：

```text
## YYYY-MM-DD：改动标题

- 目的：为什么修改。
- 文件：新增、修改、删除的准确路径。
- 内容：关键参数或代码行为。
- 验证：执行的检查及结果。
- 影响/回退：使用限制、风险和回退方法。
- 状态：已完成、部分完成或待验证。
```

仅调查、阅读文档、提出方案但未改动文件的工作，可记录为“调查结论”，不得列为“已完成改动”。
