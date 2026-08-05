# Yunle 底盘反馈与低速控制预备组件

该组件接收 Yunle JD03 底盘反馈，并包含一个默认关闭的 `0x121` 低速控制
通路。Yunle Profile 明确配置 `enable_control_send: false`，因此当前部署仍然
只接收反馈，不会创建控制命令 Reader，也不会发送 UDP/CAN 控制报文。

## 安全边界

- Profile 的控制发送开关默认为且显式设为 `false`。
- 发送开关关闭时仅绑定 UDP socket 并调用 `recvfrom`。
- 仅实现控制 ID `0x121`；`0x123` 未实现，也不会发送。
- 编译级命令硬上限为 `1.5 km/h`，实测车速超过 `1.7 km/h` 会阻断；
  转向硬上限为 `90%`，命令超时为 200 ms，失效制动发送时间为 500 ms。
  Profile 不能突破这些限制。允许 Apollo Control 的速度计算值最多高出配置
  上限 `0.05 km/h`，但实际下发值始终钳制到配置上限，不会提高线控速度。
- 控制命令必须新鲜且包含挡位、速度和转向；只接受 D、N、R 挡。
- 制动命令可能使底盘反馈 `parking_status=true`，因此控制联锁独立校验原始
  挡位；但驻车状态下仍禁止任何非零速度命令。
- 底盘核心反馈必须新鲜、最高告警等级小于 2；Profile 默认还要求实车
  自动驾驶物理开关已按下。
- 已经开始发送后若命令或联锁失效，最多发送 500 ms 的零速、回正、制动
  `0x121`，随后停止发送。
- 默认 `/apollo/canbus/chassis` 报告 `COMPLETE_MANUAL`，`engage_advice`
  报告 `DISALLOW_ENGAGE`；只有显式测试配置可在底盘反馈已处于自动驾驶模式时
  上报 `COMPLETE_AUTO_DRIVE`。
- 核心反馈超过配置的超时时间后报告 `CHASSIS_CAN_LOST`。

## 实车链路

Yunle Profile 使用以下参数：

```text
本机：192.168.1.102:8002
ETH-CAN：192.168.1.10:4002
通道：CAN2
封装：每帧 13 字节
```

实车接收报文的 Info 字节为 `0x28`，低四位表示 DLC 8。Apollo 10 参考
ETH-CAN 发送端使用 `0x08`，因此当前 Profile 暂存 `control_info_byte: 8`，
但保持发送关闭。启用前必须在静止、架空或具备可靠急停条件下验证 ETH-CAN
通信中断恢复行为，并确认本车适配器发送方向的 Info 字节。

## 通道

```text
/apollo/canbus/chassis
/apollo/canbus/yunle_chassis_detail
```

独立详情通道避免与 Apollo 各车型私有 `chassis_detail` protobuf 类型发生冲突。

## JD03 V4.5 协议说明

- 小鱼 800 轮胎直径为 266 mm，Profile 使用轮胎半径 `0.133 m`。
- `0x51` 的工作模式值 `1` 表示自动驾驶，`3` 表示遥控器模式。
- `0x77` 按 V4.5 解析 13 个连续的 3-bit 告警等级，覆盖 BMS、MCU
  和转向系统；等级 `0/1/2/3` 分别表示正常、提示、降速和紧急制动。
- `0x7F1` 最后一个 16-bit 字段是实际平均轮速 `real_speed_rpm`，
  不是目标转速。
- 当前实车行驶抓包中，`0x168` 的两个前轮 RPM 字段始终为零。Yunle
  Profile 将 `front_wheel_speed_valid` 设为 `false`，因此标准
  `Chassis` 把前轮速度标记为无效；详情通道仍保留四轮原始 RPM。
- 实车 BMS 报文与 V4.5 文档存在版本差异。当前保留经抓包验证可得到合理
  电压、电流和 SOC 的旧格式，待与仪表和扩展帧 `0x17904001` 联合核对。

## 控制协议与运行状态

`yunle_control_protocol` 是纯内存 `0x121` 数据编码器，用 V4.5 文档中的
刹车、左右转向、3 km/h 前进和倒车示例进行单元测试。编码器本身不创建
socket；运行时组件引用该编码器，但只有 `enable_control_send: true` 时才会
订阅 `/apollo/control` 并使用现有 UDP socket 发送 `0x121`。`0x123` 未实现。
`yunle_control_protocol_self_test.cc` 提供不依赖 GoogleTest 或网络下载的
标准 C++ 离线自检入口。

详情通道增加以下控制诊断字段：

```text
control_send_enabled
control_command_received
control_command_fresh
control_interlocks_ok
control_failsafe_active
sent_control_frame_count
control_send_error_count
commanded_target_speed_kph
commanded_steering_percentage
commanded_gear
control_block_reason
control_pad_start_required
control_pad_started
last_control_pad_action
planning_terminal_stop_received
planning_terminal_stop_fresh
terminal_stop_release_ready
remote_release_active
remote_release_frame_count
planning_terminal_stop_reason
terminal_stop_released_to_remote
```

当前 Profile 下应看到 `control_send_enabled: false`、发送计数为 0，并显示
`control send disabled by Profile`。不得仅为消除此提示而打开控制开关。
Apollo Control 联调配置额外打开 `require_control_pad_start: true`：在
Dreamview/Control 发送 `/apollo/control/pad` 的 `START` 之前，即使已经收到
Routing 和 Planning 轨迹，Yunle 组件也不会向 SCU 转发 `0x121` 控制帧。
该联调配置还打开 `release_to_remote_on_terminal_stop: true`：当
`/apollo/planning` 明确报告 destination/reference-end/mission-complete，
且车辆已经静止、ControlCommand 为零速时，组件等待 800 ms 后发送 500 ms 的
remote/neutral 释放帧，并解除 Pad START gate，避免短路线到终点后 Apollo
继续持有 AUTO 并周期性下发零速/刹车命令。

## 隔离的实车调试入口

正常启动文件始终使用发送关闭的配置。首次实车调试必须显式启动
`yunle_chassis_control_test.launch`，它使用独立话题
`/apollo/canbus/yunle_test_control`，不会订阅 Apollo 正常的
`/apollo/control`。

`tools/yunle_control_test_publisher.py` 默认向该隔离话题发布，并额外限制为
最高 `1.5 km/h`（约 `0.4167 m/s`）、`90%` 转向、单次最多 6 秒。每次命令
结束后会发布 500 ms
零速制动命令。正常组件和调试组件不能同时运行，因为二者绑定同一个 UDP
端口 `8002`。

Apollo 控制链路实车调试必须显式启动
`yunle_chassis_apollo_control_test.launch`。该启动文件订阅正常
`/apollo/control` 话题，并使用同样的 `1.5 km/h`、`90%`、6 秒测试限幅。
发布器可通过 `--topic /apollo/control` 复用同一套分阶段换挡、释放、运动和
停车校验逻辑，先验证 Apollo 控制话题到 Yunle receiver 的下发链路。
该测试配置额外设置 `report_auto_driving_mode: true`，只有当底盘反馈已经是
自动驾驶模式时才把 `/apollo/canbus/chassis` 上报为
`COMPLETE_AUTO_DRIVE`，用于让 Apollo Control 模块进入真实计算路径。默认
receiver 和隔离测试 receiver 均不打开该开关。

JD03 的目标速度分辨率为 `0.1 km/h`。编码器对非负请求向下量化，并用很小
的浮点容差避免 `0.3 km/h` 被误截断成 `0.2 km/h`；同时避免例如
`0.36 km/h` 被四舍五入成超过请求值和安全上限的 `0.4 km/h`。实车测试中
`1.0 km/h` 请求可能稳定反馈为 `0.9 km/h`，发布器对 `1.0 km/h` 及以上请求
接受一档以内的 SCU 反馈低偏差，较低速度请求仍按严格目标校验。

`tools/yunle_planning_test_publisher.py` 只向 `/apollo/planning` 发布轨迹，
用于验证 Planning、Apollo Control、`/apollo/control` 和 Yunle receiver 的
完整链路。默认仍生成直线；`--turn left` 或 `--turn right` 生成恒定曲率圆弧，
`--radius-m` 限制在 `[10, 100] m`，默认 `10 m`。左转曲率为正、右转曲率为负。
每次轨迹都以最新定位姿态为起点，结束时改发零速直线轨迹，避免停车阶段继续
请求转向。首次横向实车测试应使用 `1.0 km/h`、3 秒并分别测试左右方向。

车辆不能通过遥控器或实体开关预置 D/R 挡时，隔离发布器只在显式指定
`--allow-stationary-gear-change` 后允许从 N 挡请求 D/R。该模式强制目标速度和
转向为零、制动为 `100%`、实测车速为零且持续时间不超过 1 秒。换挡与非零
速度命令必须分两次执行。

如果底盘在 `0x121` 超时后自动回到 N 挡，可显式使用 `--shift-then-move`。
该模式仅允许从 N 挡开始并请求 D 挡，连续执行“零速全制动换 D、零速
释放制动、最高 `1.5 km/h` 前进不超过 6 秒”三个阶段。换挡和释放阶段
始终保持零转向；移动阶段允许 `--steering` 指定 `[-90%, 90%]` 的小角度
转向。JD03 的前后转向通道命令符号按 SCU 协议同号下发；实车机械方向
应表现为前后反向。每个阶段都会校验挡位、驻车、制动、通信和告警反馈；换挡和
释放阶段还要求实测车速近似为零，移动阶段校验组件采纳的目标速度、转向
命令和控制互锁，但不要求实测车速立刻大于零。转换条件不满足时不会进入
移动阶段，而是执行零速制动。

验证运动中的遥控刹车或急停时，需要额外指定
`--expect-hardware-brake-interrupt`。该模式把移动阶段出现
`remote_brake` 或 `emergency_brake`、目标速度归零、驻车/SCU 制动生效，并在
`settled` 与 `stability check` 阶段停稳作为通过条件。

命令失败或收到 Ctrl-C 时，工具会先尝试零速制动。由于 Apollo Python Cyber
在一次性发布器退出时可能卡在后台回调线程，工具在打印并刷新结果后会直接
结束自身进程，避免终端永久卡住。
