# Yunle JD03 车型清单

本 Profile 用于 Yunle JD03（小鱼 800）车辆在 Apollo 11.0 下的整车适配。
它组合底盘、传感器、定位、Control 参数和 Dreamview 测试入口；Apollo 11
发行包的原始算法和底层驱动源码不在本 Profile 中修改。

## 适配范围

| 项目 | 当前实现 |
| --- | --- |
| 车型/底盘 | Yunle JD03 / 小鱼 800 |
| Apollo 版本 | Apollo 11.0，包管理安装方式 |
| 底盘源码 | `modules/canbus_vehicle/yunle` |
| 底盘协议 | Yunle JD03 V4.5，UDP/ETH-CAN，控制 CAN ID `0x121` |
| 激光雷达 | 镭神 LS-C16 V4 |
| 雷达适配源码 | `modules/drivers/lidar/lslidar_c16v4_custom` |
| 室内 IMU | 轮趣/FDILink N100，Yunle Cyber 串口驱动 |
| Dreamview 辅助源码 | `modules/tools/yunle_dreamview_bridge` |
| 整车配置 | `profiles/yunle` |

## Profile 配置目录

```text
profiles/yunle/modules/
├── canbus_vehicle/yunle/   # receiver、实车测试配置、DAG 和 launch
├── common/data/            # JD03 vehicle_param 覆盖
├── control/                # Yunle 低速横纵向控制参数
├── dreamview_plus/         # Yunle 传感器、室内建图和整车测试模式
├── external_command/       # 室内导航命令转换和 RoutingResponse 输出配置
├── drivers/gnss/           # 旧 CGI-230 串口及解析配置，室内链路不再依赖
├── drivers/lidar/          # LS-C16 V4 参数和 DAG
├── localization/           # RTK 与室内 CPU NDT localization 配置
├── loam_velodyne_indoor/   # 室内激光 SLAM 兼容性测试配置
├── map/                    # 室内 Apollo HD Map
├── planning/               # 室内低速 Planning 配置和 lane-follow 任务配置
├── routing/                # 室内 HDMap Routing 配置
├── slam_local/             # 已停用的 CUDA slam_local 试验配置
└── transform/              # 静态外参配置
```

## Dreamview 模式

`Yunle Sensor Tests` 用于单独检查传感器和预览链路：

- `LslidarC16V4`
- `PointCloudBridge`
- `YunleChassisPreview`

`Yunle Vehicle Tests` 用于受控整车链路测试：

- `LslidarC16V4`
- `StaticTf`
- `Gnss`
- `RtkLocalization`
- `YunleApolloControlTest`
- `Control`

`Yunle Indoor Mapping Tests` 用于无 GNSS 位置的室内激光 SLAM
兼容性测试：

- `LslidarC16V4`
- `N100Imu`
- `IndoorTestTf`
- `IndoorSlamMapping`
- `YunleChassisPreview`

该模式不包含 `PointCloudBridge`、`RtkLocalization`、`Control` 或实车
控制 receiver。它已完成静止漂移、直行距离、左右转角和地图保存验证，当前
用于生成室内点云地图；不会直接生成 Apollo 语义 HD Map。

该模式的 `N100Imu` 使用独立的 `n100_imu_driver.dag`：直接读取 N100
串口数据，并继续发布室内 LIORF 需要的 `/apollo/sensor/gnss/imu`，同时在
收到 AHRS 姿态后发布 `/apollo/sensor/gnss/corrected_imu`。它不发布 GNSS
位置，也不发布 `world -> imu` 动态 TF，避免和 LIORF 所需的
`localization -> imu` 形成双父节点冲突。`Yunle Vehicle Tests` 中原来的
室外 GNSS/RTK 链路需要 GNSS 硬件，拆除华测设备后不能作为室内导航入口使用。

`IndoorTestTf` 以 500 ms 周期在 `/tf` 发布带当前时间戳的两段固定变换
`localization -> imu -> lslidar16v4`，分别满足 LIORF 初始化和
LaserOdometry 的传感器坐标转换；不周期重发 `/tf_static`，避免 Apollo
TF Buffer 的静态消息缓存持续增长。`localization` 明确定义在 IMU 原点且
使用相同坐标轴，因此 `localization -> imu` 按定义为单位变换，不是待测的
雷达外参；N100 设备标识为 X 向前、Y 向右、Z 向下，Y/Z 经 Yunle N100
驱动转换后发布为 Apollo/ROS 常用的 X 向前、Y 向左、Z 向上。`imu ->
lslidar16v4` 使用 JD03 尺量初值：平移 `(0.51, 0.00, 1.09) m`、无旋转。
该雷达外参来自 JD03 实车尺量和安装轴向确认，正式建图前仍需标定；
`IndoorTestTf` 不能与 `StaticTf` 同时启动。

`Yunle Indoor Localization Tests` 用于加载固定点云地图进行无 GNSS
位置的室内定位验证：

- `LslidarC16V4`
- `N100Imu`
- `IndoorTestTf`
- `IndoorLiorfOdometry`
- `IndoorNdtLocalization`
- `YunleChassisPreview`

`IndoorLiorfOdometry` 运行已通过距离和转角验证的 LIORF 链路，为固定地图
定位提供相对激光里程计；`IndoorNdtLocalization` 通过 Yunle 输入桥接器，将
相对里程计加上已验证基线使用的 `(10000, 9000000, 0) m` 室内原点，并将与其
配对的 LS-C16 V4 点云送入 Apollo 的 CPU NDT 定位模块。NDT 固定加载
`data/map_work/yunle_indoor/jd03_indoor_20260803_01/ndt_map/local_map`，输出
`/apollo/localization/pose` 和 `/apollo/localization/ndt_lidar`。

该基线已确认能够持续发布 NDT 位姿，但静止误差仍超过 1 米。
`IndoorNdtLocalizationFloatSafe` 是独立的 A/B 测试入口，使用
`(10000, 10000, 0)` 原点和 `local_map_float_safe`；其他 NDT 参数、输入点云和
激光外参均与基线一致。它与 `IndoorNdtLocalization` 发布相同的话题，任何时刻
只能启动其中一个。

`IndoorNdtLocalizationFloatSafeMinus90` 继续使用 float-safe 地图，只把在线
点云 Z 轴旋转从 `+90 deg` 改为 `-90 deg`。一帧实车点云与 `global.pcd` 的直接
对齐结果表明，`-90 deg` 的截断均方误差比 `+90 deg` 低约 7.6 倍。各 NDT
入口使用独立 DAG，但发布相同的话题，任何时刻只能启动其中一个。

`IndoorNdtLocalizationFloatSafeMinus90Map03` 使用完整记录前进、返回并越过
起点轨迹的 `jd03_indoor_20260803_03` float-safe 地图，同时保留已验证的
`-90 deg` 点云外参。该入口使用独立 bridge DAG，便于与旧地图入口独立启停。

`IndoorNdtLocalizationMap03MeasuredExtrinsic` 是 Map03 的外参 A/B 入口：地图、
输入和 NDT 参数保持不变，只将零平移外参替换为尺量的
`(0.03, -0.33, 0.67) m, -90 deg`。该项用于验证雷达中心与 IMU 定位参考点的
物理杠杆臂，不是地图坐标平移补偿。LIORF 输出已经使用 IMU/车体参考点，
因此该入口沿用原始 LIORF 位姿，不在输入 bridge 中重复应用平移外参；该入口
不能与其他 NDT 入口同时启动。

`IndoorNdtLocalizationMap03MeasuredZStable` 是在上述 measured 入口基础上的
垂向稳定 A/B 入口。Apollo CPU-NDT 仍使用相同地图、相同实测外参和相同输入
点云执行固定地图匹配，但它的原始结果先发布到
`/apollo/yunle/indoor/ndt/raw_pose` 与 `/apollo/yunle/indoor/ndt/raw_lidar`；
Yunle stabilizer 再保留 NDT 的 X/Y，使用 LIORF/bridge odometry 的
Z/roll/pitch，并把 LIORF/NDT yaw 加 `+90 deg` 转成 Apollo 车身 heading 后
发布最终 `/apollo/localization/pose` 与
`/apollo/localization/ndt_lidar`。这不是固定 Z 偏移补偿，而是避免 Apollo
CPU-NDT 在当前短室内地图上进入 3D 高度局部解；验证时仍必须同时观察
Fitness、X/Y 闭环和 Z span。该入口也不能与其他 NDT 入口同时启动。

`IndoorPclOmpNdtLocalizationMap03MeasuredZStable` 是不修改 Apollo 原生
CPU-NDT 的 Yunle 自有替换入口。它复用相同的 LIORF input bridge 和
stabilizer，但中间匹配组件改为直接加载
`jd03_indoor_20260803_03/global.pcd`，在内存中加 `(10000, 10000, 0)` 偏移后
使用从 art107 引入的 `pclomp` NDT 做整图 PCD 匹配，并继续发布同名 raw topics。
该入口用于验证 PCD+pclomp 是否比 Apollo NDT tile 链路稳定；验证通过前不要与
旧 NDT 入口同时启动。

`Yunle Indoor Navigation` 用于从 Dreamview Plus 启动已验证的室内导航链路：

- `LslidarC16V4`
- `N100Imu`
- `IndoorTestTf`
- `IndoorLiorfOdometry`
- `IndoorPclOmpNdtLocalizationMap03MeasuredZStable`
- `IndoorRouting`
- `IndoorCommand`
- `IndoorEmptyPrediction`
- `IndoorPlanning`
- `YunleApolloControlTest`
- `Control`

该模式加载 `/apollo/modules/map/data/yunle_indoor_map03_planning`，也就是
此前 Planning-only 预检通过的室内单车道 Apollo HD Map。`IndoorRouting`
使用该地图的 `base_map` 和 `routing_map`；
`IndoorCommand` 同时启动 `external_command_process` 与 `old_routing_adapter`，
把 Dreamview 的 `/apollo/routing_request` 转成 lane-follow command，再输出
Planning 使用的 `/apollo/routing_response`。NDT stabilizer 已经将室内定位
航向转换为 Apollo 车辆航向，因此 `IndoorPlanning` 和 `Control` 直接读取
`/apollo/localization/pose`，不再用 route heading 覆盖实时航向。`IndoorEmptyPrediction`
以 10 Hz 发布空的 `/apollo/prediction`，复现已验证 preflight 中的无障碍输入，
不启动完整 perception/prediction 链路。

`yunle_indoor_map03_planning` 使用稳定 PCL-OMP NDT 坐标系下的原始 LIORF
拟合中心线，不再保留旧 NDT 验证阶段加入的额外中心线平移。该调整优先修正
HDMap 与 Planning 实际定位输入的坐标系差异；车道宽度保持 `3.0 m`，不依赖
过宽车道绕过 Planning 的道路边界检查。

`common/data/vehicle_param.pb.txt` 覆盖 Apollo 默认 Lincoln MKZ 车身参数，
使用小鱼 800 `0.900 m x 0.700 m` 的车身轮廓，并按 IMU 到车头前缘
`0.550 m`、到车尾 `0.350 m`、到左边 `0.380 m`、到右边 `0.320 m`
设置 edge 参数；否则 Planning 会按
`front_edge_to_center=3.89 m` 的大车模型判断终点，短距离 route 会明显提前
停车。

室内 Planning 的固定速度参数为 `planning_upper_speed_limit=0.500 m/s` 和
`default_cruise_speed=0.500 m/s`。`destination_check_distance=20.0 m`，
用于让室内短路线从一开始就生成 destination stop fence；它不是停车误差阈值。
该模式包含
`YunleApolloControlTest`，它监听正常 `/apollo/control` 并允许发送控制帧；
因此不能和只读 `YunleChassisPreview` 或其他绑定 UDP `8002` 的 receiver
同时启动。Dreamview 操作默认保持 `Record`，需要实车控制时再显式进入
`Waypoint_Follow` 或 `Auto_Drive`，并确保场地清空、遥控急停可用。
当前已验证过 Planning-only 轨迹输出和 0 速度 Control 链路，首次非零速度
实车导航仍应按低速短时测试执行。

`YunleApolloControlTest` 在该模式下还启用终点释放保护：当 Planning 报告
destination/reference-end/mission-complete，且车辆已静止、ControlCommand
为零速时，receiver 等待 800 ms 后发送 500 ms 的 remote/neutral 释放帧，
并解除 Dreamview START gate。短路线测试结束后可用
`modules/canbus_vehicle/yunle/tools/yunle_chassis_diagnostic.py` 观察
`terminal_stop_released_to_remote` 与 `remote_release_frame_count`，确认
AUTO 已释放而不是继续周期性下发零速/刹车帧。

定位测试模式的按钮分开是为了能独立观察进程状态：先启动
`IndoorLiorfOdometry`，确认 `liorf/mapping/odometry` 存在后，再启动一个
NDT 定位入口。该模式
不能与 `IndoorSlamMapping` 同时启动，也不能启动 `PointCloudBridge`，否则
会出现重复 LIORF 进程或多个 `/apollo/localization/pose` 发布者。首次验证
只观察静止定位和小范围手推后的地图坐标恢复，不启动 Planning、Control 或
测试控制 receiver。

NDT 的在线点云外参还包含一项坐标约定转换。LS-C16 V4 的 Apollo 点云使用
`X 前、Y 左、Z 上`，室内 LIORF 保存地图使用 `X 右、Y 前、Z 上`；因此 NDT
专用外参当前将在线点云绕 Z 轴旋转 `-90 deg` 后再与固定地图匹配。该文件
目前只接入了轴向旋转，平移仍为零；正式定位时还必须按照统一的定位参考点
接入标定后的 IMU 到激光雷达平移外参。

此前的 Apollo 预编译 `slam_local` 会在组件构造阶段初始化 CUDA；当前主机
驱动低于其 CUDA Runtime 要求，即使关闭三个 GPU 算法开关也会以
`cudaErrorInsufficientDriver` 退出。因此 Dreamview 已不再引用该入口；保留
`modules/slam_local` 下的 Profile 文件仅供问题追踪，不作为可运行链路。

## 主要 Cyber 通道

```text
/apollo/sensor/lslidar16v4/Scan
/apollo/sensor/lslidar16v4/PointCloud2
/apollo/sensor/gnss/imu
/apollo/sensor/gnss/corrected_imu
/apollo/sensor/gnss/odometry
/apollo/sensor/gnss/ins_stat
/apollo/localization/pose
/apollo/canbus/chassis
/apollo/canbus/yunle_chassis_detail
/apollo/routing_request
/apollo/routing_response
/apollo/planning/command
/apollo/planning
/apollo/prediction
/apollo/control
```

其中 N100 室内驱动只负责 `/apollo/sensor/gnss/imu` 和
`/apollo/sensor/gnss/corrected_imu`；`/apollo/sensor/gnss/odometry` 与
`/apollo/sensor/gnss/ins_stat` 属于 GNSS/INS 链路，拆除华测设备后不会由
N100 提供。

## 建图与定位路线

JD03 后续按使用环境维护两条彼此独立的地图与定位链路：

1. **室内激光链路**：不依赖 GNSS 位置，在 N100 可提供 IMU 的条件下，
   使用 LS-C16 V4 完成激光 SLAM 建图和基于点云地图的定位；由点云生成
   俯视底图，再人工绘制车道、边界及连接关系，生成可供 Routing、Planning
   使用的室内 HD Map。该链路用于室内导航和教学。
2. **室外 GNSS 链路**：车辆移至具有稳定 RTK/GNSS 信号的室外场地后，使用
   GNSS/INS、激光雷达和真实场地外参完成完整建图、定位及导航流程。GNSS
   轨迹单车道地图仅作为这条链路的早期联调地图。

点云定位地图、点云俯视底图和 Apollo 语义 HD Map 是三个不同产物，必须使用
同一坐标系生成和加载。室内链路已进入 Dreamview 固化阶段；Routing、Planning
和 Control 启动前仍必须确认当前加载的是 `yunle_indoor_map03_planning` 与
`IndoorPclOmpNdtLocalizationMap03MeasuredZStable`，实车移动前必须先通过只读、
Planning-only 和 0 速度 Control 预检。

## 安全边界

- 普通 `yunle_chassis_receiver` 只接收底盘反馈，不发送控制帧。
- 实车控制只能使用显式测试 receiver；同一时刻不能启动多个绑定 UDP
  端口 `8002` 的 Yunle receiver。
- 当前实车控制硬上限为 `1.5 km/h`、转向 `90%`，命令超时为 200 ms。
- Planning 实车测试应保持低速、短时、场地清空并确保遥控急停可用。
- `Yunle Indoor Navigation` 中的 `YunleApolloControlTest` 会发送控制帧；
  只观察定位或底盘反馈时应使用 `YunleChassisPreview`，不要启动该发送使能
  receiver。
- `YunleApolloControlTest` 需要 Dreamview/Control Pad `START` 后才会向
  SCU 转发 `0x121`；发送 Routing 和看到 Planning 轨迹不再等于车辆已放行。
- `PointCloudBridge` 只用于没有正式定位时的点云预览；启动正式
  localization 或 Control 前必须关闭，避免两个发布者同时写
  `/apollo/localization/pose`。

详细协议、启动方式和测试命令见
`modules/canbus_vehicle/yunle/README_cn.md`；工程改动历史见
`helpdocs/CHANGELOG_APPLICATION_CORE.md`。
