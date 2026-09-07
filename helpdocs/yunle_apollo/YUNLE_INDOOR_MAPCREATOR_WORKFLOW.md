# Yunle 室内建图与 Map Creator HDMap 制作手册

本文档用于教学演示：遥控小车完成室内点云建图，录制 map creator
底图 record，生成 NDT 定位地图，用 map_creator 编辑 Apollo HDMap，并把
新地图安装到当前 Yunle 室内导航固定槽位。

## 固定槽位

为减少配置改动，本流程固定覆盖两个运行槽位。覆盖前脚本会自动备份，便于回退。

定位地图槽位：

```text
/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_03
```

这里放：

```text
global.pcd
3D-Pose.txt
6D-Pose.txt
slam_pose_result.bin
record/
ndt_map/local_map_float_safe/
```

HDMap 槽位：

```text
/apollo/modules/map/data/yunle_indoor_map03_planning
```

宿主机真实目录是：

```text
/home/yunle/application-core/profiles/yunle/modules/map/data/yunle_indoor_map03_planning
```

这里放：

```text
base_map.bin
base_map.txt
sim_map.bin
sim_map.txt
routing_map.bin
routing_map.txt
editor_map.json
```

当前配置已经固定读取这些槽位。正常情况下，新地图覆盖进去后，不需要修改
Localization、Routing、Planning 的 `--map_dir` 配置。

## 地图类型

这套流程会产生两类地图。

点云/NDT 定位地图：

```text
global.pcd + ndt_map/local_map_float_safe
```

用途：给 localization 做点云匹配，输出 `/apollo/localization/pose`。

Apollo HDMap：

```text
base_map.bin + routing_map.bin + sim_map.bin
```

用途：给 Dreamview 显示车道线，给 Routing/Planning 导航。

map_creator 只能制作 HDMap，不能替代 NDT 定位地图。

## 准备

以下命令建议在 Apollo 开发容器内执行，工作目录固定为：

```bash
cd /apollo_workspace
```

确认工具存在：

```bash
ls /opt/apollo/neo/bin/ndt_map_creator
ls /opt/apollo/neo/bin/editor_map_converter
ls /opt/apollo/neo/bin/tile_map_images_creator
ls /opt/apollo/neo/bin/cyber_recorder
```

确认 N100/雷达外参一致：

```bash
sed -n '1,80p' \
  profiles/yunle/modules/transform/params/yunle_indoor_test_imu_lidar_extrinsics.yaml
sed -n '1,80p' \
  profiles/yunle/modules/localization/msf/params/yunle_indoor_ndt_lidar_extrinsics_minus_90_measured.yaml
```

当前 N100 版本应为：

```text
localization -> imu: identity
imu/localization -> lslidar16v4: translation=(0.51, 0.00, 1.09), rotation=identity
```

如果 IMU 或雷达重新安装过，必须先同步这两个外参配置，再重新录图并生成
NDT 地图。旧外参生成的 NDT 地图不能可靠匹配新外参下的实时点云。

启动 map_creator 前端/后端时，另开终端：

```bash
cd /apollo_workspace/modules/map_creator/map_editor
npm run start:backend
```

再另开一个终端：

```bash
cd /apollo_workspace/modules/map_creator/map_editor
npm run start:frontend
```

浏览器访问：

```text
http://127.0.0.1:3000
```

如果前端已经 build，也可以只启动后端并访问：

```bash
bash /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_start_map_creator_backend.sh
```

脚本检测到后端已经监听成功后会输出浏览器地址。默认访问：

```text
http://127.0.0.1:58000
```

## 1. Dreamview 启动建图模块

打开 Dreamview，选择室内建图相关模式：

```text
yunle_indoor_mapping_tests
```

启动这些模块：

```text
LslidarC16V4
N100Imu
IndoorTestTf
IndoorSlamMapping
YunleChassisPreview
```

检查：

```text
激光雷达有点云
TF 正常
IndoorSlamMapping 正常运行
```

## 2. 录制 map_creator 底图 record

record 主要给 `tile_map_images_creator` 生成 map_creator 的二维底图瓦片使用。

新建本次地图目录。示例：

```bash
MAP_NAME=jd03_indoor_student_$(date +%Y%m%d_%H%M%S)
MAP_DIR=/apollo_workspace/data/map_work/yunle_indoor/$MAP_NAME
mkdir -p "$MAP_DIR/record"
echo "$MAP_DIR"
```
或者固定这个MAP_DIR路径名字
MAP_DIR=/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_new


这里允许先创建 `record/` 子目录。后面保存 LIORF 点云地图时，
`MAP_DIR` 里只能提前存在这个 `record/` 子目录；如果里面已经有
`global.pcd`、`3D-Pose.txt` 等旧地图文件，保存脚本会拒绝覆盖。

开始录制点云 record：

```bash
/opt/apollo/neo/bin/cyber_recorder record \
  -o "$MAP_DIR/record/map.record" \
  -c /apollo/sensor/lslidar16v4/PointCloud2
```

保持这个命令运行。

遥控小车慢速走完需要建图的区域。建议：

```text
沿车道中心走一遍
如果空间允许，走一个 L 型或带转弯的轨迹
不要只在原地转
```

结束录制：

```text
在 cyber_recorder 终端按 Ctrl-C
```

## 3. 保存 LIORF 点云地图

保持 Dreamview 的建图模块仍在运行。

执行：

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_indoor_map_save_test.py \
  --output-dir "$MAP_DIR" \
  --confirm-test-save
```

检查输出目录里至少有：

```bash
ls "$MAP_DIR"/global.pcd
ls "$MAP_DIR"/3D-Pose.txt
ls "$MAP_DIR"/6D-Pose.txt
ls "$MAP_DIR"/slam_pose_result.bin
```

如果 `6D-Pose.txt` 里只有 1 个点，说明建图时车基本没动。这样的 pose
不能作为 HDMap 对齐参考。后续仍可用点云做底图，但需要单独录一段
localization 校准轨迹来校正 HDMap 的旋转和平移。

## 4. 生成点云预览图

这一步不是 Apollo 运行必需，但强烈建议做。它用于人工检查 `global.pcd`
有没有明显断裂、跳变、方向异常。

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_pcd_topdown_renderer.py \
  "$MAP_DIR/global.pcd"
```

检查：

```text
$MAP_DIR/topdown_preview.png
```

如果点云形状明显不对，先不要继续做 HDMap。

## 5. 生成 NDT 定位地图

准备 NDT 输入：

```bash
python3 /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_indoor_ndt_map_prepare.py \
  --map-dir "$MAP_DIR"
```

生成 NDT 地图：

```bash
/opt/apollo/neo/bin/ndt_map_creator \
  --pcd_folders "$MAP_DIR/ndt_input_float_safe/pcds" \
  --pose_files "$MAP_DIR/ndt_input_float_safe/poses.txt" \
  --map_folder "$MAP_DIR/ndt_map/local_map_float_safe" \
  --resolution_type single \
  --resolution 0.25 \
  --resolution_z 0.25 \
  --zone_id 50 \
  --set_road_cells false \
  --pool_size 20
```

检查：

```bash
ls "$MAP_DIR/ndt_map/local_map_float_safe/config.xml"
```

也可以用一条命令完成预览、NDT 准备、NDT 生成和安装固定槽位：

```bash
python3 /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_prepare_and_install_indoor_ndt_slot.py \
  --src "$MAP_DIR"
```

如果前面已经手动生成过 `topdown_preview.png` 或
`ndt_map/local_map_float_safe/config.xml`，整合脚本会复用已有结果，
跳过对应步骤，然后继续做检查、备份和安装。

正式安装前可先 dry-run：

```bash
python3 /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_prepare_and_install_indoor_ndt_slot.py \
  --src "$MAP_DIR" \
  --dry-run
```

安装后固定槽位变成：

```text
/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_03
```

脚本会生成类似备份：

```text
/apollo_workspace/data/map_work/yunle_indoor/jd03_indoor_20260803_03.backup_YYYYmmdd_HHMMSS
```

## 6. 生成 map_creator 二维底图瓦片

先生成瓦片配置。不需要从
`modules/map_creator/tile_map_images_creator/conf/image_creator_conf.pb.txt`
复制后手工改目录，脚本会直接生成
`$MAP_DIR/image_creator_conf.pb.txt`：

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_make_map_creator_image_conf.py \
  --map-dir "$MAP_DIR" \
  --map-name "$MAP_NAME"
```

脚本运行后会打印：

```text
本次生成的配置文件
map_creator 后端读取的底图目录
下一步 tile_map_images_creator 命令
后续换目录或换硬件时优先修改的参数
```

通常只需要改：

```text
--map-dir               当前建图工作目录
--map-name              map_creator 里显示/选择的底图名
--point-cloud-channel   雷达点云 topic，默认 /apollo/sensor/lslidar16v4/PointCloud2
--localization-channel  定位 topic，默认 /apollo/localization/pose
--record-dir            record 输入目录，默认 $MAP_DIR/record
--slam-pose             SLAM 位姿文件，默认 $MAP_DIR/slam_pose_result.bin
--image-dir             map_creator 底图瓦片输出目录
--bin-dir               中间 map_bin 输出目录，默认 $MAP_DIR/map_bin
```

室内默认点云过滤参数已经在脚本里设置为：

```text
upper_height_limit_relative_to_pose: 1.5
lower_distance_limit: 0.3
```

如果更换雷达高度、安装角度或点云质量变化明显，可以通过这些参数微调：

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_make_map_creator_image_conf.py \
  --map-dir "$MAP_DIR" \
  --map-name "$MAP_NAME" \
  --upper-height-limit-relative-to-pose 1.5 \
  --lower-height-limit-relative-to-pose -1.0 \
  --upper-distance-limit 20.0 \
  --lower-distance-limit 0.3
```

保持 Dreamview 中 `IndoorTestTf` 或对应静态 TF 模块运行。`tile_map_images_creator`
离线处理 record 时仍需要查询点云 frame 到 `localization` 的静态外参。

执行：

```bash
/opt/apollo/neo/bin/tile_map_images_creator \
  -c "$MAP_DIR/image_creator_conf.pb.txt" \
  -i "$MAP_DIR/record" \
  -o "/apollo_workspace/modules/map_creator/map_editor/data/base_map/$MAP_NAME/map_images"
```

正常结束后检查：

```bash
ls "/apollo_workspace/modules/map_creator/map_editor/data/base_map/$MAP_NAME/map_images/tiles.json"
```

如果命令长时间不退出但持续写文件，可以等待。若确认卡死，可另开终端：

```bash
pkill -f tile_map_images_creator
```

## 7. 用 map_creator 编辑 HDMap

打开 map_creator 页面：

```text
http://127.0.0.1:58000
```

如果没有使用已构建前端的后端服务，而是单独启动了开发前端，则访问
`http://127.0.0.1:3000`。

基本操作：

```text
选择底图：选择 $MAP_NAME
新建或打开标注地图
绘制车道边界、车道、停止线等语义元素
保存标注地图
发布地图
```

发布成功后，产物默认在：

```text
/apollo_workspace/modules/map_creator/map_editor/data/released_map/<发布地图名>/
```

其中应包含：

```text
editor_map.json
base_map.bin
base_map.txt
sim_map.bin
sim_map.txt
routing_map.bin
routing_map.txt
```

## 8. 安装 HDMap 到固定槽位

示例：

```bash
RELEASED_MAP=/apollo_workspace/modules/map_creator/map_editor/data/released_map/yunle_indoor_new_planning
```

先 dry-run：

```bash
python3 /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_install_released_hdmap_float_safe.py \
  --src "$RELEASED_MAP" \
  --dry-run
```

正式安装：

```bash
python3 /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_install_released_hdmap_float_safe.py \
  --src "$RELEASED_MAP"
```

脚本会：

```text
把 map_creator 原始大 Y 坐标转换到 float-safe 坐标
重新生成 base_map/sim_map/routing_map
生成 routing_test.pb.txt/default_end_way_point.txt
备份旧 yunle_indoor_map03_planning
覆盖固定 HDMap 槽位
```

备份目录类似：

```text
/apollo_workspace/profiles/yunle/modules/map/data/yunle_indoor_map03_planning.backup_YYYYmmdd_HHMMSS
```

安装后重启 Dreamview，让 Map/Routing/Planning 重新读地图：

```bash
aem bootstrap restart --plus
```

## 9. 校正 HDMap 与定位坐标

安装后重新启动 Dreamview 室内导航模式，确认车和地图是否对齐。

如果车辆定位准，但车在 HDMap 里偏左、偏右或车道线斜，说明 HDMap
和 localization 坐标系之间还有旋转/平移偏差。

判断：

```text
整张地图方向歪：旋转偏差
方向对但整体偏：平移偏差
一头对一头不对：可能底图或建图本身有局部变形
```

纠正原则：

```text
不要改 localization
不要改 TF 来迁就 HDMap
应该对 map_creator 的 editor_map.json 整体做 2D 旋转和平移
```

如果建图时车沿车道中心走了几米，可以用 `6D-Pose.txt` 或 `3D-Pose.txt`
作为参考轨迹。若建图时车基本原地没动，需要单独录一段 localization
校准轨迹：

```bash
CALIB_DIR="$MAP_DIR/calib_record"
mkdir -p "$CALIB_DIR"

/opt/apollo/neo/bin/cyber_recorder record \
  -o "$CALIB_DIR/localization.record" \
  -c /apollo/localization/pose
```

遥控车沿车道中心走 1 到 2 米，空间允许时走 L 型。结束时按 Ctrl-C。

这段轨迹用于计算 HDMap 的 `rotate_deg`、`translate_x`、`translate_y`。

自动估计参数：

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_estimate_hdmap_alignment.py \
  --map-dir "$MAP_DIR" \
  --released-map "$RELEASED_MAP"
```

脚本默认读取：

```text
$MAP_DIR/6D-Pose.txt
$MAP_DIR/gnss-map-offset.txt
$RELEASED_MAP/editor_map.json
```

如果 `6D-Pose.txt` 不存在，会尝试 `3D-Pose.txt`。脚本会把建图 pose
转换到 float-safe 的 `(10000, 10000)` 坐标附近，再和 map_creator
发布地图里的车道中心线做匹配，输出：

```text
rotate_deg
translate_x
translate_y
可直接复制执行的安装命令
```

如果知道建图时车辆走的是哪一条 lane，可以指定 lane，减少误匹配：

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_estimate_hdmap_alignment.py \
  --map-dir "$MAP_DIR" \
  --released-map "$RELEASED_MAP" \
  --lane-id 1
```

注意：如果 pose 轨迹基本是一条直线，脚本能较好估计车道方向和横向偏移，
但沿车道前后方向的平移约束较弱。空间允许时，校准轨迹尽量走 L 型；
如果只能走直线，执行脚本输出的安装命令后仍需要在 Dreamview 里确认，
必要时小幅微调 `translate_x` 或 `translate_y`。

如果已经知道修正量，可以重新安装 HDMap 时直接传入：

```bash
python3 modules/tools/yunle_dreamview_bridge/yunle_install_released_hdmap_float_safe.py \
  --src "$RELEASED_MAP" \
  --rotate-deg -9.6 \
  --translate-x -0.15 \
  --translate-y 3.86
```

含义：

```text
--rotate-deg   逆时针为正，顺时针为负
--translate-x  最终整体 X 平移，单位 m
--translate-y  最终整体 Y 平移，单位 m
```

现场调试时建议每次只小幅调整，安装后重启 Dreamview/Map/Routing/Planning
观察效果。最终应以实车定位与车道中心重合为准。

## 10. Dreamview 验证导航

选择室内导航模式：

```text
yunle_indoor_navigation
```

先在 Dreamview 的地图选择处确认当前地图是：

```text
yunle_indoor_map03_planning
```

先不要启动 `Control` 和 `YunleApolloControlTest`。确认定位、HDMap、
Routing、Planning 都正常后，再进入闭环控制。

启动：

```text
LslidarC16V4
N100Imu
IndoorTestTf
IndoorLiorfOdometry
IndoorPclOmpNdtLocalizationMap03MeasuredZStable
IndoorPlanningHeadingAdapter
IndoorRouting
IndoorCommand
IndoorEmptyPrediction
IndoorPlanning
```

需要闭环控制时再启动：

```text
YunleApolloControlTest
Control
```

检查：

```text
Dreamview 能看到新 HDMap
车的位置落在正确车道中心附近
Routing 能生成路线
Planning 能生成轨迹
Control 未启动前车不会自动运动
```

## 回退

回退定位地图：

```bash
cd /apollo_workspace/data/map_work/yunle_indoor
mv jd03_indoor_20260803_03 jd03_indoor_20260803_03.bad_$(date +%Y%m%d_%H%M%S)
mv jd03_indoor_20260803_03.backup_YYYYmmdd_HHMMSS jd03_indoor_20260803_03
```

回退 HDMap：

```bash
cd /apollo_workspace/profiles/yunle/modules/map/data
mv yunle_indoor_map03_planning yunle_indoor_map03_planning.bad_$(date +%Y%m%d_%H%M%S)
mv yunle_indoor_map03_planning.backup_YYYYmmdd_HHMMSS yunle_indoor_map03_planning
```

回退后重启 Dreamview、Localization、Routing、Planning。

## 常见问题

地图能显示，但车明显不在车道中间：

```text
HDMap 和 localization 坐标没对齐。需要对 editor_map.json 做整体旋转和平移。
```

地图完全不显示，只看到车：

```text
通常是 HDMap 坐标仍在 y=9000000 附近，Dreamview float 精度不够。
请使用 yunle_install_released_hdmap_float_safe.py 安装。
也要确认 Dreamview 当前选择的地图是 yunle_indoor_map03_planning。
```

导航模式按钮已启动，但没有车辆定位：

```text
先确认 /apollo/localization/pose 是否有实时数据。
如果 NDT 日志持续出现 accepted=0，说明 NDT 匹配未通过，不会发布定位。
优先检查 NDT 外参是否和当前 N100/雷达静态 TF 一致，并确认固定 NDT 槽位
是用当前硬件外参重新录制的数据生成的。
```

定位有了，但 Dreamview 车头方向和车道垂直：

```text
通常是旧 GNSS/LIORF 坐标时代的 90 度航向补偿还在生效。
当前 N100 版本不应再额外加 90 度。
修改 C++ 后必须重新 buildtool build，并重启
IndoorPclOmpNdtLocalizationMap03MeasuredZStable。
```

定位有了，车身和车道平行，但车头车尾正好反了：

```text
最终 /apollo/localization/pose 还需要相对 /apollo/yunle/indoor/ndt/raw_pose
补偿 180 度。当前 N100 版本使用 kApolloVehicleHeadingOffsetRad = kPi。
修改 C++ 后必须重新 buildtool build，并重启
IndoorPclOmpNdtLocalizationMap03MeasuredZStable。
```

Map 模块加载成功，但日志提示 lane width 小于 half vehicle width：

```text
map_creator 里画的车道宽度太窄，或 Apollo 当前使用的 half_vehicle_width
仍是默认 1.05m。教学小车车道建议先画宽一些；若确实是窄车道，需要再统一
修正 Apollo 使用的车辆宽度参数。
```

`tile_map_images_creator` 报静态 TF 查询失败：

```text
保持 Dreamview 的 IndoorTestTf/静态 TF 模块运行。
确认点云 frame_id 有到 localization 的静态外参。
```

`6D-Pose.txt` 只有一个点：

```text
建图时车基本没动。它不能作为车道方向参考，需要单独录 localization 校准轨迹。
```

`npm run ci:all` 因 lock 文件不一致失败：

```text
在 map_editor 本地开发可改用 npm run install:all。
```

## 教学建议

第一次上课建议按这个顺序演示：

```text
1. Dreamview 启动建图模块
2. 录 record
3. 遥控车走一小段
4. 保存 global.pcd
5. 生成 topdown_preview.png
6. 生成 NDT 地图并安装固定槽位
7. 生成 map_creator 底图瓦片
8. 在 map_creator 画一条简单车道
9. 发布并安装 HDMap
10. Dreamview 验证车和车道是否对齐
```
