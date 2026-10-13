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
| 室内导航调试 | 终点释放/蠕行/到达判据/低速转向/限速 | 已按诊断调整 4 处配置 + 1 处源码（已重建安装），实车复验待执行 |
| 室外 RTK | CGI-230 室外建图/导航配置 | GNSS 配置、JD03 室外 TF/外参与室外采图模式（含底图脚本 RTK 模式）均已就绪；RTK 固定解、航向与 UTM 坐标已完成端到端验证，可进入采图 |
| CGI-230 设备参数 | 安装/杆臂/航向基线（主副天线接反补偿） | 已修正九项安装参数并 `saveconfig` 持久化，断电复验与航向实测（1.2°）通过；设备未进入"组合导航模式"，不影响 Apollo 室外定位 |

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

## 24. 2026-09-06 至 2026-09-07：N100 室内链路、室内导航稳定性与 MapCreator 流程更新

- 目的：在拆除华测 GNSS/INS 后接入轮趣/FDILink N100，按新的 IMU 坐标系更新
  室内激光定位和建图链路；同时整理 MapCreator 制图流程、室内 HD Map 安装和
  导航调试工具，并针对低速导航中的航向、停车转向和 Control 消息积压问题增加
  诊断与保护。
- 代码：新增
  `modules/tools/yunle_dreamview_bridge/n100_imu_driver.cc` 及
  `dag/n100_imu_driver.dag`，直接读取 N100 串口二进制帧，发布
  `/apollo/sensor/gnss/imu` 和 `/apollo/sensor/gnss/corrected_imu`；新增
  `yunle_n100_imu_diagnostic.py` 检查 IMU 频率、加速度、角速度和姿态；新增
  `yunle_estimate_hdmap_alignment.py`，根据保存的 `6D-Pose.txt`/
  `3D-Pose.txt` 和 MapCreator 车道中心线估计 HD Map 的二维旋转、平移安装参数。
  新增 `yunle_start_map_creator_backend.sh`，启动 MapCreator 后端并等待
  `58000/healthz` 就绪后打印访问地址。
- 代码：更新室内定位和 TF：
  `indoor_ndt_pose_stabilizer.cc` 将 N100 坐标约定下的 Apollo 车辆航向补偿由
  `+90 deg` 改为 `180 deg`；`indoor_test_tf_publisher.cc` 使用尺量外参
  `imu -> lslidar16v4 = (0.51, 0.00, 1.09) m`、单位旋转；
  `indoor_planning_heading_adapter.cc` 增加 route 几何航向、原始/输出航向日志，
  便于排查规划线与定位航向不一致。室内导航 HMI 模式已移除该 Heading Adapter，
  由 NDT stabilizer 输出的车辆航向直接供 Planning 和 Control 使用。
- 代码：更新 Yunle 底盘 receiver 和配置协议，给 `/apollo/control` reader 增加
  可配置队列；增加低速/制动时清零转向、终点剩余路径门槛和对应的编译上限校验。
  当前 Profile 配置为 `control_command_pending_queue_size: 200`、
  `zero_steering_below_speed_kph: 0.35`、
  `terminal_stop_max_path_remain_m: 0.2`，用于降低 Control 消息积压、停车后
  保持大舵角以及过早触发终点释放的风险。
- 代码：`yunle_indoor_map_save_test.py` 改为保存地图前清理输出目录中除
  `record/` 外的文件和目录，同时保留 `record/` 内容；清理过程会打印删除项。
  `yunle_make_map_creator_image_conf.py` 支持点云高度/距离过滤、瓦片分辨率、矩阵
  编号和 worker 数参数，并打印目录、硬件 topic、输出位置和下一步瓦片生成命令。
  导航预检和快照工具同步切换为 N100，并兼容没有 `DrivingAction` 枚举的消息版本。
- 配置：`Yunle Indoor Localization Tests`、`Yunle Indoor Mapping Tests` 和
  `Yunle Indoor Navigation` 从旧 `GnssImu` 切换为 `N100Imu`。室内 TF 和 NDT
  外参统一为 N100/LS-C16 V4 的 `(0.51, 0.00, 1.09)`、单位旋转；Planning 改为
  直接读取 `/apollo/localization/pose`，短路线的
  `destination_check_distance` 调整为 `1.0 m`。新增
  `profiles/yunle/modules/calibration/data/yunle_jd03_xiaoyu800/vehicle_param.pb.txt`。
- 配置：`profiles/yunle/README.md`、`helpdocs/yunle_apollo/YUNLE_INDOOR_MAPCREATOR_WORKFLOW.md`
  已补充 N100 接线/外参前置条件、MapCreator 后端启动、参数化底图配置生成、HD
  Map 自动对齐、float-safe 地图安装、地图重载以及先验证定位/地图/规划再开启
  Control 的操作顺序。
- 配置：当前工作区还包含
  `profiles/yunle/modules/drivers/gnss/conf/gnss_conf.pb.txt` 和
  `profiles/yunle/modules/transform/conf/static_transform_conf.pb.txt` 的
  NovAtel 配置变化（`/dev/novatel*`、`G320N`、`world -> novatel` 等）。这部分
  不属于室内 N100 链路，提交前必须确认是否为本轮有意修改；若机器只保留 N100，
  室内模式不应依赖这些室外 GNSS 配置。
- 地图产物：MapCreator 发布了
  `modules/map_creator/map_editor/data/released_map/yunle_indoor_new_planning/`
  的 `base_map`、`sim_map`、`routing_map` 和 `editor_map.json`；固定地图槽位
  `profiles/yunle/modules/map/data/yunle_indoor_map03_planning/` 的对应文本/二进制
  地图、`default_end_way_point.txt`、`routing_test.pb.txt` 和 `editor_map.json`
  已被更新。安装过程保留了多个带时间戳的旧地图备份，便于回退；新地图是否
  纳入提交应只选择最终版本，不要把所有编辑器历史快照一起加入。
- 验证：已完成 Python 脚本语法、MapCreator 后端启动脚本语法以及工作流命令路径
  的静态检查；已在现场确认 N100 模式、室内 TF、定位/规划链路和地图发布流程
  的调试方向。修改 C++ 后仍需要在容器内重新构建并重启实际模块，不能仅凭源码
  或配置文件判断实车运行结果。
- 不纳入 Git：根目录下的 `*.INFO`、`*.log.INFO.*`，
  `modules/map_creator/map_editor/dumps/`，地图槽位下的
  `*.backup_*`/`*.bad_translate_backup_*` 目录，以及编辑器运行时生成的临时
  快照和缓存。它们是运行日志、诊断输出或回退副本；地图最终产物是否提交需
  与代码提交分开审核。
- 状态：N100 室内接入、流程工具和当前配置修改已完成；C++ 重编译后的全链路
  实车回归、NovAtel 配置变化的归属确认以及最终地图提交范围仍待提交前核对。

## 25. 2026-10-11：室内导航"走走停停"诊断、终点释放、低速转向与限速调整

- 目的：定位并缓解云乐 JD03 室内低速导航中"走走停停、走得极慢、到达终点后不释放遥控"的问题。
- 文件：修改
  `profiles/yunle/modules/control/controllers/lon_based_pid_controller/conf/controller_conf.pb.txt`、
  `profiles/yunle/modules/planning/planning_component/conf/yunle_indoor_planning.conf`、
  `profiles/yunle/modules/canbus_vehicle/yunle/conf/yunle_chassis_apollo_control_test_conf.pb.txt`、
  `modules/canbus_vehicle/yunle/yunle_chassis_receiver_component.cc`（源码改动，需重新 `buildtool build`）。
- 内容：
  - `speed_itfc_speed_cmd: 0.10 → 0.04`。终点蠕行速度原为 `0.10 m/s`，高于接收端"终点零速"
    判据 `kTerminalStopZeroSpeedCommandThresholdMps = 0.05 m/s`，使
    `terminal_stop_release_ready_` 永远无法成立、终点不释放；降到 `0.04` 后满足该判据。
  - `--passed_destination_threshold=0.3`（新增；原为默认 `0.05`）。放宽 Planning 的"到达目的地"判定。
    复验表明该项**不是**本次症状的约束条件：实测 `distance_destination` 最小 `0.00724`，本已低于原
    阈值，却未改变"终点停车"状态；保留待继续观察。
  - `terminal_stop_max_path_remain_m: 0.2 → 1.5`。接收端终点释放的剩余路径门槛；实测
    `path_remain` 最低仅 `0.332 m`，从未进入 `0.2`，使释放条件恒不成立。`1.5` 在编译期上限
    `kAbsoluteTerminalStopMaxPathRemainM = 5.0` 之内。
  - `zero_steering_below_speed_kph: 0.35 → 0.01`（先改为 0.05，随后按现场要求改为 0.01）。接收端在
    命令速度 ≤ 该值时把前/后转向指令强制清零（`yunle_chassis_receiver_component.cc:781-785`）。
    第二轮实车复验中 `plan_maxv` 全程 ≤ 0.186 m/s、命令速度大量时间低于 0.35 km/h，转向因此在
    最需要转弯的低速段被抹掉，表现为"转弯幅度不足、冲出车道"。取 0.01 后该低速限制实际等同关闭；
    "刹车时清零转向"由 `brake_enable` 分支继续保证，停车时不会保持大舵角。
  - `maximum_control_speed_kph: 1.5 → 3.0`，并配套修改
    `modules/canbus_vehicle/yunle/yunle_chassis_receiver_component.cc`：
    `kAbsoluteControlCommandSpeedLimitKph: 1.5 → 4.0`、`kAbsoluteMeasuredSpeedLimitKph: 1.7 → 4.5`，
    同时把"命令速度超过配置上限"的行为由**整条丢弃**改为**钳制并告警**。
    原因：原实现 `return false` 会让该帧掉进失效保护分支并下发 500 ms 刹车，一旦站点误差修正把
    请求速度顶过上限，就会在巡航段表现为顿挫。改为钳制后速度上限只起"调速器"作用，不再丢帧。
  - `--planning_upper_speed_limit` / `--default_cruise_speed`: `0.500 → 0.833`（m/s，即 3 km/h）。
    仅提升接收端上限不会让车真的跑到 3 km/h，必须同时抬高规划的速度上限；
    实测 3 km/h = 0.833 m/s，与接收端 3.0 km/h 一致。
- 验证：各项配置改动已确认在车端软链路径（`/apollo/modules/**` → `/apollo_workspace/profiles/current`）
  可见，无需重新执行 `aem profile use`；均需重启对应模块后生效。
  源码改动已通过 `buildtool build -p modules/canbus_vehicle/yunle` 编译并安装成功
  （exit=0，新库 mtime 2026-10-11 20:45）。**限速 3.0 的生效已实测验证**：以
  `maximum_control_speed_kph: 3.0` 启动接收端成功，启动日志为
  `Hard limits: 3 km/h and 90% steering`——若编译期上限仍是 1.5，`ValidateConfig()`
  会直接拒绝启动，故该结果反证新常量已生效。实车复验待执行。
- 影响/回退：全部改动都可用以下命令精确回退，互不影响，也不会触动本次会话之外的既有未提交改动
  （`gnss_conf.pb.txt`、`static_transform_conf.pb.txt`、19:23 安装的地图均不在此列）。
  1) 源码与配置（4 个 tracked 文件）：
     `cd /home/yunle/application-core && git checkout -- \`
     `modules/canbus_vehicle/yunle/yunle_chassis_receiver_component.cc \`
     `profiles/yunle/modules/control/controllers/lon_based_pid_controller/conf/controller_conf.pb.txt \`
     `profiles/yunle/modules/planning/planning_component/conf/yunle_indoor_planning.conf \`
     `profiles/yunle/modules/canbus_vehicle/yunle/conf/yunle_chassis_apollo_control_test_conf.pb.txt`
  2) 已安装的运行库：本次构建前的备份保存在容器内
     `/opt/apollo/neo/lib/modules/canbus_vehicle/yunle/libyunle_chassis_receiver_component.so.backup_20261011_203757`；
     回退即 `cp -a <该备份> libyunle_chassis_receiver_component.so`。**注意顺序：若把配置退回
     `maximum_control_speed_kph: 1.5`，库可以不回退；但若把库回退成旧版，则必须同时把配置退回 1.5**，
     否则 `ValidateConfig()` 会拒绝启动。
  3) 本文档：`git checkout -- helpdocs/CHANGELOG_APPLICATION_CORE.md`（本文件在本次会话前无未提交改动，
     回退只会移除本节）。
  4) 残余风险：`control_command_timeout_ms` 仍受编译期上限 `kAbsoluteControlCommandTimeoutMs = 200U`
     限制，本次未改动；如需放宽必须再次改常量并重建。
- 调查结论（未改动任何代码，仅记录）：
  - 本轮诊断所依据的运行使用的是 **2026-10-11 19:23 新安装的室内地图**：
    `base_map`/`routing_map`/`sim_map`/`default_end_way_point.txt`/`routing_test.pb.txt` 全部更新，
    终点 waypoint 由 `(9998.17, 9999.65)` 移至 `(10002.07, 9999.43)`，约 3.9 m。该图上
    `stop_s`(6.826) 比 `end_s`(6.608) 大 0.218 m（停车墙落在参考线之外）稳定复现；
    上一轮旧图上的值为 6.866 / 6.738（相差 0.128 m）。
  - 20 秒实测采样（`/apollo/control` 的 `simple_lon_debug`）：`is_stop_reason_by_destination`
    1944/1944 为真；`speed_reference` 中位 `0.000 m/s`、最大 `0.093 m/s`；`path_remain`
    0.332~1.190 m；`brake_cmd` 恒为 0；`is_full_stop` 0/1944。即车辆全程处于"终点停车"决策中，
    前进量来自纵向 PID 的站点误差 `speed_offset`，而非巡航速度。刹车无死区抖动与
    限速冲突（巡航 1.8 km/h vs 接收端拒收 1.55 km/h）在本轮均未被触发。
  - 接收端 `yunle_chassis_receiver_component.cc:852-858` 的释放判据
    `ControlCommandIsTerminalZeroSpeed()` 同时要求 `path_remain ≤ terminal_stop_max_path_remain_m`
    且命令速度 ≤ `0.05 m/s`，二者与上面的实测值叠加即构成终点死锁。
  - Apollo 11 发行包缺陷（未修改源码）：`planning_base/common/reference_line_info.cc` 的
    `SDistanceToRefEnd()` 先解引用 `ref_end_ptr->DebugString()` 再判断 `!ref_end_ptr`，该条件恒为假，
    函数恒返回 `DBL_MAX`；因此 `passed_referenceline_end_threshold = 1.5 m` 这条到达兜底实际是死代码。
  - 另发现 `dreamview_plus`（`teleop_tool`、`simulation_world` 节点）也注册了
    `/apollo/control` 的 writer；实车导航期间使用 Dreamview 遥控/仿真面板会与控制模块争抢同一控制话题。
  - 第二轮实车采样（`/apollo/control` + `/apollo/planning` 同步约 290 s）：
    `plan_maxv` 全程 `0.054~0.186 m/s`，`total_path_length` 仅 `0.46~1.24 m`；行驶段
    `is_stop_reason_by_destination` 转为 0（说明车真正跑起来时规划会给正常轨迹），`brake_cmd`
    全程为 0。轨迹短小的原因见下一条。
  - **轨迹为何只有约 1 米（根因，代码级）**：
    `planning/pnc_map/lane_follow_map/lane_follow_map.cc:226-228` 在
    `next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1` 时置
    `stop_for_destination_ = true`；该标志经 `planning_base/common/frame.cc:158-160` 成为
    `frame->is_near_destination()`，而终点停车墙仅在它为真时才创建
    （`traffic_rules/destination/destination.cc:63-65`），墙的位置为
    `routing_end->s() - virtual_stop_wall_length(0.1) - stop_distance(0.2)`
    （`destination.cc:118-124`）。室内路由只有起点/终点两个途经点，因此**车辆一旦越过第 1 个
    途经点，"下一个途经点"即为最后一个，`stop_for_destination_` 从此恒为真**：终点墙每帧重建、
    轨迹被截断到墙前，于是全程只有约 1 米轨迹、速度上限被压在 `0.1~0.19 m/s`。
  - `destination.cc:81-84` 自身带有告警：`dest_sl.s() + front_edge_to_center > reference_line.Length()`
    会 "cause ego is stoped by PATH_END fence not by destination"。本工程实测
    `stop_s 6.826 > end_s 6.608`、车头偏移 `0.550 m`，正落在该告警条件内——车辆实际被
    **路径末端挡墙**停住而非被"终点"停住，这是 `ReachedDestination()` 与 `mission_complete`
    无法稳定成立的直接原因。
  - 第二轮采样中 `/apollo/planning` 在"0.46 m 终点停车轨迹"与"空轨迹（`total_path_length = 0`）"
    之间反复切换；空轨迹会清除接收端的 `planning_terminal_stop_received`
    （`planning_terminal_stop_timeout_ms: 1000`），使 `terminal_stop_release_delay_ms: 800` 的连续
    静止计时反复清零。因此即使 `terminal_stop_max_path_remain_m` 放宽到 1.5，本轮仍未发生释放
    （全程 `release_ready=False`、`terminal_released_to_remote=False`、`remote_release_frame_count=0`）。
    释放未成立的第二个条件是站点误差 `speed_offset` 把命令速度顶到 `0.3~1.2 km/h`，高于
    `kTerminalStopZeroSpeedCommandThresholdMps = 0.05 m/s`。
  - 待验证的零风险实验：在 Dreamview 里下发**带中间途经点**的 routing。此时大部分路程中
    `next_routing_waypoint_index_ != size()-1`，`stop_for_destination_` 保持 false，终点墙不建，
    轨迹长度与巡航速度应恢复正常。若实验成立即确认上述根因，治本方案为
    "重画地图使终点落在参考线以内" 或 "回退到 19:23 之前的地图版本"。
- 状态：4 项配置改动已完成；转弯不足（`zero_steering_below_speed_kph`）与终点释放两项实车复验待执行；
  "轨迹过短"的根因已定位到代码级，治本方案（多发途经点 / 重画终点 / 回退地图）待确认。

## 26. 2026-10-11：室外 RTK 建图与导航准备（GNSS 配置、JD03 室外 TF 与外参、室外采图模式）

- 目的：按 `helpdocs/yunle_apollo/YUNLE_OUTDOOR_RTK_PLAN.md` 的方案，为 CGI-230 RTK 室外建图与导航
  做配置准备（该方案文档中的 C-1 ~ C-4）。
- 文件：
  - 修改 `profiles/yunle/modules/drivers/gnss/conf/gnss_conf.pb.txt`（由外来 NovAtel 残留恢复为华测配置）
  - 修改 `profiles/yunle/modules/transform/conf/static_transform_conf.pb.txt`（Apollo 默认 velodyne64 +
    相机链路 → JD03 室外最小链路）
  - 新增 `profiles/yunle/modules/transform/params/yunle_outdoor_localization_lidar_extrinsics.yaml`
  - 新增 `profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_outdoor_mapping.pb.txt`
  - 修改 `modules/tools/yunle_dreamview_bridge/yunle_make_map_creator_image_conf.py`（新增 RTK 模式）
- 内容：
  - GNSS（C-1）：`format: HUACE_TEXT`；串口
    `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_DWCRm153609-if00-port0`、460800；
    三条 `login_commands`（`unlog com1 HCINSPVATZCB` / `unlog com1 GPCHC` / `log com1 GPCHC ontime 0.01`）；
    `rtk_solution_type: RTK_RECEIVER_SOLUTION`；`imu_type: CPT_XW5651`；`proj4_text` 为 UTM **zone 50** WGS84；
    `tf { frame_id: "world" child_frame_id: "imu" enable: true }`；`use_gnss_time: true`。
    内容与 git 提交版本一致（Apollo 11 的华测解析器只吃 `$GPCHC`/`$GPCHCX` 文本，不解析二进制）。
  - TF（C-2）：`static_transform_conf.pb.txt` 收敛为单条 `localization → lslidar16v4`，外参文件指向新增的
    室外 yaml。RTK 模式下 `tile_map_images_creator` 需要把点云变换到 `localization` 帧，故这条静态变换是必需的。
    **刻意不添加** `localization → imu`：`imu` 的动态父节点由 GNSS 驱动发布（`world → imu`），再加一个静态
    父节点会让 TF 树出现双父。
  - 外参（C-2，已定稿）：以 `helpdocs` 之外的 Yunle Autoware 标定文档
    （`~/下载/tmptobedeleted/YUNLE_AUTOWARE_CHANGELOG_AND_VALIDATION.md`，用户确认与当前实车安装一致）为基准。
    该文档给出三个实测 frame（均为 `base_link` = 后轴中心，X 前 / Y 左 / Z 上，无旋转）：
    `cgi230/imu_link (-0.03, 0.27, 0.21)`、`cgi230/gnss_link (0.59, -0.26, 1.06)`（主天线）、
    `lidar_link (0.50, 0.00, 1.30)`。由此得
    **`CGI-230 → 雷达 = (0.53, −0.27, 1.09) m`**，旋转取单位阵（文件
    `yunle_outdoor_localization_lidar_extrinsics.yaml`）。
    与 2026-10-11 的独立尺量路径（`N100 → 雷达 (0.51, 0, 1.09)` 叠加 `N100 → CGI-230 (0.02, 0.26, 0)`）
    所得 `(0.49, −0.26, 1.09)` 相差 x 4 cm、y 1 cm、z 0 cm；采用标定值。
  - 采图模式（C-3）：新增 `yunle_outdoor_mapping`，模块为 `LslidarC16V4`、`StaticTf`、`Gnss`、
    `RtkLocalization` 与只读的 `YunleChassisPreview`，`operations: Record`（不含 Control 与发送使能 receiver）。
  - 底图脚本 RTK 模式（C-4）：`yunle_make_map_creator_image_conf.py` 新增 `--mode {slam,rtk}`，默认 `slam`
    以保持室内流程不变。`--mode rtk` 生成 `slam_mode_selection_conf { enable_slam_mode: false }`
    （`tile_map_images_creator` 的 RTK 分支，用 record 里的 `localization_channel` 位姿拼底图），
    跳过 `slam_pose_result.bin` 的存在性校验，并把"需要 StaticTf 提供 `localization -> lslidar16v4`"
    打印在操作提示里。注意 `SlamModeSelectionConf` 的两个字段在 proto 里都是 `required`，因此
    `slam_pose_path` 仍会写入，只是 RTK 模式下不被读取（生成文件里已用注释说明）。
- 验证：C-4 脚本已完成四项静态验证：
  （1）`ast.parse` 语法检查通过（不产生 `__pycache__`）；
  （2）`--mode slam --dry-run` 输出 `enable_slam_mode: true`，与改动前一致；
  （3）`--mode rtk --dry-run` 输出 `enable_slam_mode: false` 并带说明注释，提示文案切换到 StaticTf；
  （4）反向验证：对只有 `record/`、没有 `slam_pose_result.bin` 的目录，`--mode slam` 按预期报
  `slam pose file not found`，`--mode rtk` 正常通过——证明校验已按模式区分。
  其余验证：`gnss_conf.pb.txt` 恢复后与 git 提交版本逐字节一致（`git status` 中该文件已不再是 modified）；
  `static_transform_conf.pb.txt`、两个新增文件在车端均确认可读（`sed` 读取内容正常）。
  两个**新增**文件不在既有软链中（车端 `/apollo/modules/**` 是逐文件软链，新增文件不会自动出现），
  已按既有格式手工创建软链，目标均指向 `/apollo_workspace/profiles/current/modules/...`。
  室外实车联调尚未开始。
  **CGI-230 链路实测验证（2026-10-11 21:27 起，先室内、后移至室外）**：
  - 启动方式：`mainboard -p gnss -d /apollo/modules/drivers/gnss/dag/gnss.dag`。室内验证时先停掉了
    `n100_imu`、`yunle_indoor_liorf_odometry`、`yunle_indoor_pclomp_ndt_localization_map03_measured_z_stable`
    与 `IndoorTestTf`，原因见下方"话题冲突"一条。
  - **链路与解析全部正常**：`/apollo/sensor/gnss/imu`、`corrected_imu`、`odometry`、`ins_stat`
    均实测 **98.1 Hz**；`best_pose` **1.1 Hz**（与第 6.4 节一致，华测 `best_pose` 为固定 1 Hz 节流）。
  - **串口输出正确**：`/apollo/data/gpsbin/<时间戳>.bin` 中为 100 Hz **纯 `$GPCHC` ASCII 文本、带合法
    校验和、无二进制混杂**，证明配置中的 `unlog com1 HCINSPVATZCB` + `log com1 GPCHC ontime 0.01`
    确实生效（第 6.2 节的设备侧切换逻辑工作正常）。
  - **日志中的 `HUACE ASCII message is not complete` 是正常现象**：`huace_text_parser.cc:85-88` 要求
    累积缓冲以 `0x0A` 结尾，分片读取时必然出现该告警，解析器会把碎片累积到下一批，不是故障。
  - **室内（无卫星）实测**：`NSV1/NSV2 = 00/00`、`Status = 00`、`Lat/Long` 为**空字段**（两个连续逗号）、
    `ins_status = COLD_START(6)`、`sol_type = NONE(0)`、`pos_type = 0`——这是"收不到卫星"的必然结果，
    不是设备故障。
  - **室外（有卫星）原始报文实例**：
    `$GPCHC,2440,49170.240,315.16,-2.28,-0.70,-0.01,0.03,0.03,0.0127,-0.0359,0.9942,40.06425325,116.36222957,34.79,0.05,0.06,-0.01,0.07,22,24,91,1,0002*42`
    字段对照（顺序：GPSWeek,GPSTime,Heading,Pitch,Roll,gyro×3,acc×3,Lat,Long,Alt,Ve,Vn,Vu,V,NSV1,NSV2,
    Status,Age,warning）：`acc_z = 0.9942 g`（重力正确、IMU 正常）、`Lat/Long/Alt` 有值、
    卫星 **22/24**。Apollo 侧同一时刻为 `sol_status = SOL_COMPUTED`、`sol_type = NARROW_FLOAT(34)`、
    `latitude/longitude = 40.0642559 / 116.3622386`、`height_msl = 35.70 m`。
  - **`$GPCHC` 的 `Status` 字段是十六进制文本**，高半字节=卫星状态、低半字节=系统状态（手册 4.1 节，
    第 22 项）：`42` = 高 4（RTK 稳定解定位定向）+ 低 2（组合导航模式），即历史验证过的目标状态；
    本次在遮挡处实测为 `91` = 高 9（RTK **浮点**解、定位不定向）+ 低 1（卫导模式）；把车推到开阔处后
    **30 秒内即收敛为 `42`**，`sol_type = INS_RTKFIXED(56)`，其后 5 分钟持续稳定（卫星 25~27、
    Age 0~1 s、定位重复性厘米级）。**结论：先前的浮点解是遮挡/多路径造成的，不是设备或配置问题。**
  - **航向通路已验证正确**：同一时刻对比设备 `$GPCHC` 的 `Heading`（北 0°、顺时针）与
    `/apollo/localization/pose` 的 `heading`：`(90 − 299.810) mod 360 = 150.190`，与 Apollo 实测
    heading **误差 0.000°**。这证明 GNSS 驱动 `PublishOdometry` 中硬编码的 `−90°` 对本设备是
    **正确**的；方案文档中的"待确认项 2（航向 −90° 疑点）"据此关闭。
  - `/apollo/localization/pose` 实测正常：UTM `x=445614.829 y=4435085.865 z=34.130`，与原始报文的
    经纬高自洽；静止速度 0；`roll=−1.11° pitch=+0.51°`（停于微斜坡）。
  - **仍有一处内部不一致（待处理）**：`pose.heading = 150.190°` 而 `pose.euler_angles.z = 60.190°`，
    两者恒定相差 **90.000°**。heading 已证正确，故问题在 `euler_angles` 一路（由 `PublishCorrimu` 的
    `euler.z = ins.euler.z − 90°` 填入）。使用 `euler_angles` 的消费者会拿到偏 90° 的航向，
    需逐个确认 Planning / Control / map_creator 各自读取哪个字段。
  - 坐标量级提示：UTM `y ≈ 4.435e6`，float32 在该量级间隔约 0.5 m，Dreamview 显示地图时可能出现
    抖动或不显示（方案文档 §3.3 已标注），必要时采用统一平移。
  - **由几何推导出的设备端安装参数（华测约定 X 右 / Y 前 / Z 上）**，可用于核对或写入设备：
    ```text
    ins2antoffset    0.53 0.62 0.85     # IMU→主天线杆臂（模长 1.18 m，横向即占 0.53 m）
    ant2bodyoffset  -0.26 -0.59 -1.06   # 主天线→后轴中心
    headingoffset    0.0  0.0 140.9     # 主→副天线基线夹角（副天线在主天线后 0.64 m、左 0.52 m）
    setbaseline      0.82 0.10          # 双天线基线长度 0.825 m
    wheeltread       ?     0.52         # 轮距待实测；轴距取 vehicle_param 的 0.52 m
    ```
  - **外参仍有一个待确认的前提**：Apollo 的 `localization` 帧是 CGI-230 的**输出点**，由设备端
    `ANT2OUTPOSOFFSET` 决定（0 = IMU 点，1 = 主天线，2 = 指定目标点）。当前外参文件按"输出点为
    IMU 点"编制；若设备实际输出到后轴中心或其他点，则需改用从该点量起的平移（例如后轴中心→雷达
    为 `(0.50, 0.00, 1.30) m`）。该值需通过 `getting gilccfg` 回读确认。
  - `best_pose.num_sats_* = 0` 属已知的 Apollo 11 华测解析器限制（未把 `NSV1/NSV2` 写入该字段，
    见第 6.4 节）；卫星数应直接读原始报文的 `NSV1/NSV2`。
  - **话题冲突（必须遵守的操作约束）**：室外链路必须使用 CGI-230 **成套**提供 `imu`/`corrected_imu`/
    `odometry`/`ins_stat`/`best_pose`，因此启动 `Gnss` 前必须停掉 `N100Imu` 驱动（二者写同一对话题），
    并且要停掉 `IndoorTestTf`（它给 `imu` 提供 `localization` 父节点，而 GNSS 驱动会发布 `world → imu`，
    同时存在会造成 TF 双父）。
  - 驱动运行期间会把原始报文持续写入 `/apollo/data/gpsbin/`，是排查数据质量与差分解状态的第一手材料。
  - 注意：本次 `Gnss` 是命令行手动启动的（`mainboard -p gnss`），**Dreamview 不掌握该进程**；
    若之后要从 Dreamview 启动 `Yunle Outdoor Mapping` 模式，需先手动 `kill` 掉该实例，
    否则两个实例会抢占 `/dev/ttyUSB0`。
- 关于 `aem profile use`（本次踩到的坑，供后续参考）：该命令依赖 `APOLLO_CONF_PATH` /
  `APOLLO_ENV_WORKROOT` 等环境变量。在容器里用 `bash -lc` 调用且未 `source /opt/apollo/aem/activate.sh`
  时，`APOLLO_CONF_PATH` 为空，`conf_manager.py` 会报 `dir [] not exists!`；
  而 source 之后这两个变量又会被解析成**宿主路径**（`/home/yunle/application-core/.aem/...`），
  在容器内不存在，报 `profiles dir not found`。宿主上则因 `/usr/bin/sudo` 的 setuid 位异常而无法执行
  （`sudo: /usr/bin/sudo 必须属于用户 ID 0 并且设置 setuid 位`）。结论：**新增 profile 文件时，
  要么在容器内用不带 `bash -lc` 的方式执行 `aem profile use yunle`，要么按既有格式手工补软链**；
  本次采用后者。
- 影响/回退：
  - 回退 GNSS 配置：`git checkout -- profiles/yunle/modules/drivers/gnss/conf/gnss_conf.pb.txt`
    （注意：回退后得到的是外来的 NovAtel 残留版本，**不是可用配置**，仅用于还原工作区状态）。
  - 回退 TF 配置：`git checkout -- profiles/yunle/modules/transform/conf/static_transform_conf.pb.txt`。
  - 删除新增文件：`rm profiles/yunle/modules/transform/params/yunle_outdoor_localization_lidar_extrinsics.yaml
    profiles/yunle/modules/dreamview_plus/conf/hmi_modes/yunle_outdoor_mapping.pb.txt`。
  - 删除车端软链：`rm /apollo/modules/transform/params/yunle_outdoor_localization_lidar_extrinsics.yaml
    /apollo/modules/dreamview_plus/conf/hmi_modes/yunle_outdoor_mapping.pb.txt`。
  - **室内链路不受影响**：室内模式使用 `yunle_indoor_test_static_transform_conf.pb.txt`（`IndoorTestTf`）
    与 `N100Imu`，本次均未改动；`static_transform_conf.pb.txt` 只被室外/整车测试模式里的 `StaticTf` 使用。
- 待现场确认（直接影响外参正确性）：
  1. 设备端 `ANT2OUTPOSOFFSET` 配的是哪个输出点。若设备把位置输出到后轴中心/车辆目标点而不是 IMU
     测量中心，则 `localization` 帧原点不是 CGI-230 的 IMU 中心，本文件的外参必须从该点重新量测。
  2. GNSS 驱动的 `PublishOdometry` 对 `$GPCHC` 航向硬编码了 `−90°`（`data_parser.cc:290-295`）。上车后
     必须核对 `/apollo/localization/pose` 的 heading 与实际朝向；若差 90°，说明两个坐标系之间存在旋转，
     本文件的平移分量也必须相应旋转。
  3. `N100 → 雷达 = (0.51, 0.00, 1.09)` 仍是尺量初值，未正式标定；雷达或任一 IMU 重新安装后都需重测。
- 状态：配置准备完成；室外外参为**待现场核对的初值**；室外实车联调未开始。

## 27. 2026-10-11：CGI-230 设备端安装参数修正（天线接反补偿）

- 目的：室外 RTK 采图前排查发现，设备内保存的安装/杆臂参数与实际整车几何严重不符，会让定位输出带上
  一个**随航向旋转的约 0.94 m 偏移**，并让航向整体偏差。不先改对设备参数，采出的底图必然扭曲。
- 设备（非工作区文件，记录以备追溯）：
  - 华测 CGI-230，SN `5886062`，固件 `update_CGI230_M_v1.4.5_b20250808`
  - 差分：`SWAS_STATE:OPEN COOR:WGS84 LOGIN SUCCESS`、`RTK MODE:SWAS TYPE:TCP CONNECTED
    IP:192.168.200.185 PORT:12003`（SWAS，且坐标系已是 WGS84，无需切换）
  - 接口：COM1 / COM2 均为 460800
- 判定依据（为什么按"主副天线接反"处理）：
  1. 用户现场确认：**定位主天线插在设备左侧 `GNSS2` 接口上**；手册 2.2 节规定
     `GNSS 1（Fakra）：定位天线`、`GNSS 2（Fakra）：定向天线`，故障表第 10 条写明
     "设备正面右侧为 GNSS1 接口，左侧是 GNSS2 接口"，并把该情形列为"双天线接反"，
     现象正是"标定过程中输出的状态位显示浮点解定位不定向"——与本次在遮挡处实测到的
     `Status=91（RTK 浮点解定位不定向）`相符。
  2. 数值比对：设备旧值 `ANT2BODYOFFSET:0.29 -0.68 -0.30` 的横向分量为正、
     `INS2ANTOFFSET:-0.30 -0.03 0.09` 的前向分量接近 0，只有"接反"假设推出的值与之同号；
     不接反假设下横向符号相反。
- 内容：经用户授权，通过 COM1 修正九项参数并 `saveconfig`。几何基准取已验证的 Yunle Autoware 标定
  （`base_link` = 后轴中心，X 前 / Y 左 / Z 上）：`cgi230/imu_link (-0.03, 0.27, 0.21)`、
  `cgi230/gnss_link (0.59, -0.26, 1.06)`、`lidar_link (0.50, 0, 1.30)`，以及用户实测的副天线相对
  主天线位置（后 0.64 m、左 0.52 m、同高）。按"接反"（设备认定的主天线 = 物理上位于
  (-0.05, 0.26, 1.06) 的那根）换算到华测约定（X 右 / Y 前 / Z 上）后写入：

  ```text
  insangle 0 0 0 5 5 5
  setheadingbase car
  setattitudebase car
  headingoffset 0.0 0.0 -39.1 5 5 5
  ant2bodyoffset 0.26 0.05 -1.06 1 1 1
  ins2antoffset 0.01 -0.02 0.85 1 1 1
  ant2outposoffset 0 0 0 0          # 输出点由 "2 指定目标点" 改为 "0 IMU 点"
  bodytype 2 0 0
  wheeltread 0.462 0.500            # 轮距取底盘规格书 462 mm；轴距由 0.46 改为规格书 500 mm
  saveconfig
  ```

  修改前后对照（`getting gilccfg` 回读）：

  | 参数 | 修改前 | 修改后 |
  | --- | --- | --- |
  | `HEADINGOFFSET` | 0.00 0.00 **180.00** | 0.00 0.00 **-39.10** |
  | `ANT2BODYOFFSET` | **0.29 -0.68 -0.30** | **0.26 0.05 -1.06** |
  | `INS2ANTOFFSET` | **-0.30 -0.03 0.09** | **0.01 -0.02 0.85** |
  | `WHEELTREAD` | 0.46 **0.46** | 0.46 **0.50** |
  | `ANT2OUTPOSOFFSET` | **2** 0.29 -0.68 -0.30 | **0** 0.00 0.00 0.00 |
  | `INSANGLE` / `BODYTYPE` / `POS_MEM` | 0 0 0 / 2 0 0 / CLOSE | 不变 |
  | `BASELINE` | 0.00 0.00（未设置） | **刻意未改**：航向本已正常解出，未写入未实测校核的 0.82 m，避免破坏已工作的解 |

  - **参数与物理安装的对应关系（改动天线接线前必读）**：本组参数是按"**设备认定的主天线 =
    物理上位于左后方那一根**"推导的。实测安装（车体坐标，原点 = 后轴中心，X 前 / Y 左 / Z 上）：

    | 硬件 | 安装位置 | 车体坐标 (m) |
    | --- | --- | --- |
    | **主天线（定位）** | **左后** | `(−0.05, +0.26, 1.06)` |
    | **副天线（定向）** | **右前** | `(0.59, −0.26, 1.06)` |
    | CGI-230 IMU | 车体中部偏左 | `(−0.03, +0.27, 0.21)` |

    由此得到的三个几何参数及其含义：

    | 参数 | 写入值 | 几何含义 |
    | --- | --- | --- |
    | `HEADINGOFFSET` | **−39.1** | 主(左后) → 副(右前) 基线相对车头指向**右前方** −39.1° |
    | `ANT2BODYOFFSET` | **0.26 0.05 −1.06** | 主天线 → 后轴中心（横 0.26 / 前后 0.05 / 高 1.06 m） |
    | `INS2ANTOFFSET` | **0.01 −0.02 0.85** | IMU → 主天线（横向与前后近似重合、高 0.85 m） |

    **注意"天线安装位置"与"接线端口"是两件事**：
    - **位置**：与上表一致 ✔（主天线在左后、副天线在右前）
    - **接线**：定位主天线的**线插在设备左侧 `GNSS2` 端口**，而手册 2.2 节规定 `GNSS1`（右侧）
      才是定位天线 → 按手册算属于"接反"
    - **本组参数已把这一接线事实一并补偿进去**，因此功能正确（航向实测误差 1.2°，见"验证"第 4 条）

    **若将来有人改动天线接线（把线换到别的端口），本组参数立即失配** ✗，需改用 Case 1 组
    （对应"定位天线接 GNSS1 / 主天线在前右"的布置）并重新做航向验证：

    ```text
    headingoffset  0.0  0.0  140.9  5 5 5
    ant2bodyoffset -0.26 -0.59 -1.06 1 1 1
    ins2antoffset   0.53  0.62  0.85 1 1 1
    ant2outposoffset 0
    saveconfig
    ```

- 验证：
  1. 九条命令与 `saveconfig` 均回 `$command,[...],ok`，回读值即上表"修改后"。
  2. **断电重启后再次回读，九项参数全部保持** → `saveconfig` 持久化成功。
  3. 位置输出发生约 1 m 位移（lat +0.25 m、lon −0.69 m、**alt −0.6 m**），与"修掉 0.94 m 旋转偏移、
     其中垂直分量 0.76 m"的预期一致。
  4. **航向实测**：车头朝东（用户指南针确认 ≈90°）时设备报 `Heading = 88.75~88.76°`，误差约 1.2°。
  5. Apollo 侧端到端复核：`best_pose.sol_type = INS_RTKFIXED(56)`、`ins_stat.pos_type = 56`；
     `/apollo/localization/pose` 的 UTM 与设备经纬度用独立 UTM 实现比对为
     ΔE 0.19 m / ΔN 0.03 m / Δz 0.00 m；航向 `(90 − 设备Heading) mod 360` 与 Apollo `heading`
     一致到 0.4°。
  6. 遗留：设备 `Status` 低半字节仍为 `0（初始化）`，未进入 `2（组合导航模式）`。经查
     `huace_base_parser.cc` 的 `PrepareMessageStatus`：`sol_type` 只由**高半字节**（卫星状态，
     当前 4 = RTK 稳定解定位定向）决定，故 Apollo 侧 `sol_type = INS_RTKFIXED`；而 `rtk_localization`
     只检查 `pos_type`、不检查 `sol_status`，**因此不影响室外定位与采图**。若要恢复 INS 桥接能力
     （GNSS 短时丢失时靠惯导续推），需以 >0.5 m/s 持续行驶 30~60 s 让其完成组合导航对准。
  7. **航向验证方法（判断"参数与安装是否匹配"的唯一可靠办法，以后动过天线必做）**：
     把车头对准一个**已知方向**（手机指南针读数即可），在**有固定解**（`Status` 高半字节 = 4）时读
     设备 `$GPCHC` 的 `Heading`，或读 `/apollo/localization/pose` 的 `heading`
     （注意后者以东向为 0、逆时针为正，换算关系 `heading = (90 − 设备Heading) mod 360`）：

     | 实测偏差 | 结论 | 处理 |
     | --- | --- | --- |
     | **1~2°** | 参数与安装**匹配** | 无需处理（2026-10-11 实测：车头朝东 90°，设备报 88.76°） |
     | 约 **39°** | `HEADINGOFFSET` 与天线布置不符（多半是主副天线角色判断反了） | 换成另一组 Case 的 `HEADINGOFFSET`（−39.1 ↔ 140.9） |
     | 约 **180°** | 主副天线整体反了 | 检查接线端口与两组参数的对应关系 |

     **注意**：该验证**必须在室外有固定解时做** —— 无卫星（`Status=00`、`NSV=00/00`）时
     `Heading` 无效，不能用来判断。
  8. **断电保留验证（2026-10-12 补做）**：给 CGI-230 断电、等待后重新上电，再次 `getting gilccfg`
     回读：**九项参数全部保持**，且 COM1 仍以 **100 Hz** 输出 `$GPCHC`（"输出 GPCHC"这条持久化
     同样保留）→ 证明 `saveconfig` 已把参数与输出配置真正写入非易失存储 ✔
- 影响/回退：
  - **回退到改动前的原始设备值**（已实测可用）：
    ```text
    headingoffset  0.00  0.00  180.00  5 5 5
    ant2bodyoffset  0.29 -0.68 -0.30 0.05 0.05 0.05
    ins2antoffset  -0.30 -0.03  0.09 0.05 0.05 0.05
    ant2outposoffset 2 0.29 -0.68 -0.30
    wheeltread 0.46 0.46
    saveconfig
    ```
  - **操作历史（务必知悉，避免误读本文档）**：本次设备侧先后发生五次操作，本节上文"内容"记录的是
    第 ① 次：
    1. **2026-10-11 首次写入**修正参数 + `saveconfig`（即上文"内容"表格）
    2. 同日 **回退**到改前原值（使用上面的回退命令块，已实测可用、回读确认）
    3. 同日 **再写回**修正参数 —— 回退后设备同样停在 `Status=40`，未能进入组合导航；而参数与安装的
       几何对应关系已被航向实测（1.2°）证实正确，故按用户要求写回修正版
    4. 同日 **二次写入**修正参数并回读确认（幂等操作，无副作用）
    5. **2026-10-12 断电验证**：重新上电后回读，九项参数与 GPCHC 输出配置均保留 ✔

    **最终状态 = 修正参数已持久化**（与第 ① 次写入内容一致）。
  - **关于"未进入组合导航（`Status` 低半字节 = 0）"的结论**：回退到原值后设备**同样停在 `40`**，
    说明二者**无直接因果关系**；且 2026-10-12 的监视记录显示，当时的"标定驾驶"**速度仅
    0.15~0.56 m/s**，低于手册 §7.2.2 对低速车载模式要求的 **>0.5 m/s**，这才是未完成标定的
    直接原因。后续需按手册（>0.5 m/s、连续约 5 分钟、全程固定解）重新标定后再下结论。
  - **外参不受天线接法影响**：输出点已改为 IMU 点，`localization → lslidar16v4 = (0.53, −0.27, 1.09)`
    继续有效——外参只取决于设备输出哪个点，与主副天线如何接无关。
  - **副作用（需知悉）**：`saveconfig` 会把设备当时的**运行态配置整体存盘**，因此原本持久化的
    `HCINSPVATZCB` 二进制输出（Autoware 流程依赖）被改成了 `$GPCHC` 文本（已实测：上电即输出
    GPCHC 100 Hz）。对 Apollo 无影响（驱动启动时本就要切到 GPCHC）；**若仍需 Autoware，恢复二进制**：
    ```text
    unlog com1 GPCHC
    log com1 HCINSPVATZCB ontime 0.01
    saveconfig
    ```
  - 本次为**设备持久配置**改动，不涉及工作区文件；工作区侧的配置与源码改动见第 26 条。
- 状态：已完成并验证。含**二次写入**与**断电保留验证**（2026-10-12）；最终生效值 = 上文"修改后"列。
  唯一遗留是设备未进入"组合导航模式"（经查与参数取值无直接因果，原因是标定驾驶速度未达
  >0.5 m/s，见"影响/回退"最后一条），不影响 Apollo 室外定位与采图。

## 28. 2026-10-11：清理 data/map_data 下的失效软链备份目录（消除 aem profile 报错）

- 目的：`aem profile use yunle` 会执行 `conf_manager.py recover`（调用点 `aem-profile:82`），该步骤要求
  扫描树内不存在失效软链。`data/map_data` 下 21 个地图备份目录中含 188 条失效软链，导致 recover 返回
  失败并以 `[FATAL] recover invalid conf failed.` 中止 profile 切换。本次把备份目录移出扫描树以消除该报错。
  （`aem bootstrap start --plus` 不走该步骤，故此前未阻塞 Dreamview 启动。）
- 位置与文件（均为数据目录，不涉及工程源码或 Profile 配置）：
  - 移出前：`data/map_data/yunle_indoor_map03_planning.backup_*` 21 个（含 188 条失效软链），
    以及 `yunle_indoor_map03_planning.backup_20261011_210118`、
    `yunle_indoor_map03_planning.bad_translate_backup_20260906_205437` 2 个（链接完好），共 23 个目录
  - 移出后：`map_data_backups/`（工作区根目录，与 `data/` 同级，位于 AEM 扫描树 `/apollo` **之外**）
- 内容与判据：
  - `APOLLO_CONF_PATH = ${APOLLO_ENV_ROOT}/apollo`（`activate.sh:114`），容器内 `/apollo` 即宿主的
    `.aem/envroot/apollo`，因此 recover 的扫描范围是**整棵 `/apollo` 树**。把备份留在 `data/` 下的任何
    位置仍会被扫到（第一次移到 `data/map_data_backups` 即因此无效），必须移到 `/apollo` 之外。
  - 失效链接成因：备份目录内保存的是指向 `profiles/current/modules/map/data/<备份目录名>/...` 的软链，
    而 profile 中并不存在同名备份文件——地图安装/备份工具留下的残留（时间戳自 2026-08-12 至 2026-10-11）。
  - `conf_manager.py:82-107` 的 recover 仅在"恢复源文件存在"时才执行 `os.remove` + `os.symlink`；本次
    188 条全部命中 `continue` 分支，**该报错自始至终没有删除或改写任何文件**。
- 验证：
  1. 用与 recover 相同的 `os.walk(target, followlinks=False)` 复现扫描：移出前 `/apollo` 下失效软链
     188 条（全部位于 `data/map_data_backups`），移出后 **0 条**。
  2. 生效地图 `data/map_data/yunle_indoor_map03_planning` 的 9 个文件全部可读（9/9），
     `find -xtype l` 计数 0；`profiles/current` 树失效软链同样为 0。
  3. `data/map_data` 现仅含 `borregas_ave`、`demo`、`dumps` 与本车生效地图 `yunle_indoor_map03_planning`。
  4. 移动后 23 个目录完整保留于 `map_data_backups/`，目录属主已改为 yunle，便于日后回退操作。
- 影响/回退：
  - **未删除任何数据**，仅改变位置。回退命令（完全复原）：
    `mv /home/yunle/application-core/map_data_backups/* /home/yunle/application-core/data/map_data/`
    ——注意复原后 `aem profile use` 会再次因这 188 条失效软链报错。
  - `map_data_backups/` 在工作区根目录、未被 git 跟踪（`.gitignore` 未忽略该名称，会出现在 `git status`
    的未跟踪列表中）。
  - 本次改动不影响地图内容、不影响室内导航与室外采图链路。
- 状态：已完成。

## 29. 后续记录格式

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
