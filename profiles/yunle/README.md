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
| GNSS/INS | 华测 CGI-230，Apollo `HUACE_TEXT` 解析器 |
| Dreamview 辅助源码 | `modules/tools/yunle_dreamview_bridge` |
| 整车配置 | `profiles/yunle` |

## Profile 配置目录

```text
profiles/yunle/modules/
├── canbus_vehicle/yunle/   # receiver、实车测试配置、DAG 和 launch
├── control/                # Yunle 低速横纵向控制参数
├── dreamview_plus/         # Yunle Sensor Tests / Vehicle Tests
├── drivers/gnss/           # CGI-230 串口及解析配置
├── drivers/lidar/          # LS-C16 V4 参数和 DAG
├── localization/           # RTK localization 配置
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

## 主要 Cyber 通道

```text
/apollo/sensor/lslidar16v4/Scan
/apollo/sensor/lslidar16v4/PointCloud2
/apollo/sensor/gnss/corrected_imu
/apollo/sensor/gnss/odometry
/apollo/sensor/gnss/ins_stat
/apollo/localization/pose
/apollo/canbus/chassis
/apollo/canbus/yunle_chassis_detail
/apollo/planning
/apollo/control
```

## 安全边界

- 普通 `yunle_chassis_receiver` 只接收底盘反馈，不发送控制帧。
- 实车控制只能使用显式测试 receiver；同一时刻不能启动多个绑定 UDP
  端口 `8002` 的 Yunle receiver。
- 当前实车控制硬上限为 `1.5 km/h`、转向 `90%`，命令超时为 200 ms。
- Planning 实车测试应保持低速、短时、场地清空并确保遥控急停可用。
- `PointCloudBridge` 只用于没有正式定位时的点云预览；启动正式
  localization 或 Control 前必须关闭，避免两个发布者同时写
  `/apollo/localization/pose`。

详细协议、启动方式和测试命令见
`modules/canbus_vehicle/yunle/README_cn.md`；工程改动历史见
`helpdocs/CHANGELOG_APPLICATION_CORE.md`。
