# 室外 RTK 建图操作说明（Yunle JD03 / 小鱼800）

> ## ⚠️ 状态声明：本文档对应的工作**尚未跑通**（2026-10-13）
>
> 截至本版本，室外建图只验证到 **"能启动 → 能录制 → 能生成底图瓦片 → 能在 map_creator 里看"**，
> 但 **还没有产出过一张可用的室外底图**，也 **没有跑完"画车道 → 发布 → 安装 → 导航"这条闭环**。
>
> **已知的缺口（详见第 12 节）**：
> 1. 录制时长不足（实测只有 36 s / 53 s，需要 5 分钟以上）→ 底图覆盖几乎为零；
> 2. 移动时 RTK 固定解不稳（60~98%），位姿精度不足会让点云重影；
> 3. CGI-230 始终停在 `Status=40`（初始化），未进入组合导航（`42`），原因待查；
> 4. `global.pcd` / 点云地图、NDT 兜底定位**均未实现**（属可选路线，见第 13 节）。
>
> 因此：**本文档的步骤是"设计意图 + 已验证片段"，不是"已被完整验证的操作手册"**。
> 室内流程的完整验证版见附件《MapCreator的使用和NDT地图的生成&导航》
> 与 `helpdocs/yunle_apollo/YUNLE_INDOOR_MAPCREATOR_WORKFLOW.md`。
>
> 参考：`helpdocs/yunle_apollo/YUNLE_OUTDOOR_RTK_PLAN.md`（§7~§9 方案与风险清单）

---

## 0. 原理一句话（与室内最本质的区别）

| | 室内 | 室外（RTK） |
| --- | --- | --- |
| 位姿从哪来 | **SLAM 算出来**（激光 + IMU 里程计） | **CGI-230 的 RTK/INS 解算直接给出** |
| 坐标系 | SLAM 局部坐标系（起点 = 原点） | **UTM / WGS84 绝对坐标** |
| 需不需要回环 | **需要**（SLAM 靠回环消漂移） | **不需要**（绝对定位） |
| 需不需要存 SLAM 地图 | **需要**（`global.pcd`、`slam_pose_result.bin` 等） | **不需要** |

> **室内是"用激光把位姿算出来"，室外是"用全球定位把位姿拿过来"。**
> 所以室外少了三件事：SLAM 保存、NDT 定位地图、建图起点约束；
> 多了三件事：**设备安装参数核对、固定解监视、UTM 大坐标精度处理**。

---

## 1. 第一次准备（只做一次）

### 1.1 编译建图工具（容器内）

```bash
aem enter
cd /apollo_workspace
source /opt/apollo/neo/setup.sh
bash modules/map_creator/patch_buildtool_skip_dirs.sh
buildtool build -p modules/map_creator/map_tool modules/map_creator/tile_map_images_creator
```

### 1.2 map_editor 前后端（容器内，容器自带 Node 22）

```bash
cd /apollo_workspace/modules/map_creator/map_editor
source ~/.nvm/nvm.sh
nvm use 22
npm run ci:all      # 安装依赖（只需一次）
npm run build       # 构建前端（只需一次）
```

> ⚠️ **必须在容器内执行**。宿主机只有 Node 24，而
> `yunle_start_map_creator_backend.sh` 里硬写了 `nvm use 22`，在宿主机跑会直接失败。

---

## 2. 每次开工前的设备检查（约 3 分钟，强烈建议）

室外方案的位姿全部来自 CGI-230，**设备不对，图必错**。

```bash
# 串口被 Gnss 驱动占用时无法直读；需先在 Dreamview 里停掉 Gnss 模块，
# 再直连 /dev/ttyUSB0（460800 8N1）发送命令
getting gilccfg
```

应看到（2026-10 已定稿、`saveconfig` 持久化、并通过断电复验）：

```text
INSANGLE:0.00 0.00 0.00 5.00 5.00 5.00
HEADINGOFFSET:0.00 0.00 -39.10 5.00 5.00 5.00
ANT2BODYOFFSET:0.26 0.05 -1.06 1.00 1.00 1.00
INS2ANTOFFSET:0.01 -0.02 0.85 1.00 1.00 1.00
WHEELTREAD:0.46 0.50
BODYTYPE:2 0 0
ANT2OUTPOSOFFSET:0 0.00 0.00 0.00
POS_MEM:CLOSE
BASELINE:0.00 0.00
```

**参数与物理安装的对应关系**（改动天线接线前必读）：

| 硬件 | 安装位置 | 车体坐标 (m)（原点 = 后轴中心，X 前 / Y 左 / Z 上） |
| --- | --- | --- |
| **主天线（定位）** | **左后** | `(−0.05, +0.26, 1.06)` |
| **副天线（定向）** | **右前** | `(0.59, −0.26, 1.06)` |
| CGI-230 IMU | 车体中部偏左 | `(−0.03, +0.27, 0.21)` |

- `HEADINGOFFSET = −39.1` = 主(左后) → 副(右前) 基线相对车头指向右前方 −39.1°
- `ANT2BODYOFFSET` = 主天线 → 后轴中心
- `INS2ANTOFFSET` = IMU → 主天线
- `ANT2OUTPOSOFFSET = 0` = 位置输出到 **IMU 点** ← **Apollo 外参 `(0.53, −0.27, 1.09)` 的前提**

⚠️ **"天线安装位置"与"接线端口"是两件事**：当前定位主天线的线插在设备**左侧 `GNSS2`** 端口，
而手册规定 `GNSS1`（右侧）才是定位天线 —— 上面这组参数**已把该接线事实一并补偿**（航向实测误差 1.2°）。
**若将来有人改动接线，本组参数立即失配**，需换成另一组：

```text
headingoffset  0.0  0.0  140.9  5 5 5
ant2bodyoffset -0.26 -0.59 -1.06 1 1 1
ins2antoffset   0.53  0.62  0.85 1 1 1
ant2outposoffset 0
saveconfig
```

**航向验证（唯一可靠的判断办法）**：车头对准已知方向，读 `/apollo/localization/pose` 的 `heading`
（换算关系 `heading = (90 − 设备Heading) mod 360`）：

| 实测偏差 | 结论 | 处理 |
| --- | --- | --- |
| 1~2° | 参数与安装**匹配** | 无需处理（2026-10-11 实测：车头朝东 90°，设备报 88.76°） |
| 约 39° | `HEADINGOFFSET` 与天线布置不符 | 换成 −39.1 ↔ 140.9 另一组 |
| 约 180° | 主副天线角色整体反了 | 查接线端口 + 换另一组参数 |

> 该验证**必须在室外有固定解时做**；无卫星（`Status=00`、`NSV=00/00`）时 `Heading` 无效。

---

## 3. 启动环境与室外采图模式

```bash
aem start
aem enter
cd /apollo_workspace
aem bootstrap start --plus
# 浏览器 http://localhost:8888
```

Dreamview 里选 **`Yunle Outdoor Mapping`**，打开 **5 个模块**：

| 模块 | 作用 |
| --- | --- |
| `LslidarC16V4` | 激光雷达（建图数据源） |
| `StaticTf` | 广播 `localization → lslidar16v4` 外参（**RTK 模式必需**） |
| `Gnss` | CGI-230 驱动（占用 `/dev/ttyUSB0`） |
| `RtkLocalization` | 发布 `/apollo/localization/pose`（**位姿来源**） |
| `YunleChassisPreview` | 只看底盘状态（**只观察，不发控制**） |

⚠️ **启动顺序（本工程实测坑）**：**先** `aem bootstrap start --plus`（Dreamview + monitor 起来），
**再**做其它操作。顺序不对时 Dreamview 的模块开关会**全部点不动**，日志刷
`hmi_worker.cc:1856] Running time error: monitor is not turned on!`
（成因：Dreamview 等 monitor 的 `system_status`，monitor 等 Dreamview 的 `hmi_status`，互相等待；
本次实测用注入消息也没打通）。规避：按顺序启动；万一卡住就重启 bootstrap/机器。

### 数据链路确认

```bash
cyber_monitor -c /apollo/sensor/lslidar16v4/PointCloud2   # ~10 Hz
cyber_monitor -c /apollo/localization/pose                # ~100 Hz
cyber_monitor -c /apollo/sensor/gnss/best_pose            # ~1 Hz
cyber_monitor -c /apollo/sensor/gnss/ins_stat             # sol_type / pos_type
```

判据：
- `best_pose.sol_type` 必须是 **`INS_RTKFIXED`(56)** 或 `NARROW_INT`(50)；浮点解只给 WARNING
- `localization/pose` 有数据，UTM 坐标合理（本场地约 `x≈445615, y≈4435085`）
- `StaticTf` 日志出现 `Broadcast static transform, frame id [localization], child frame id [lslidar16v4]`

---

## 4. 录制目录准备

```bash
MAP_NAME=jd03_outdoor_$(date +%Y%m%d_%H%M%S)
MAP_DIR=/apollo_workspace/data/map_work/yunle_outdoor/$MAP_NAME
mkdir -p "$MAP_DIR/record"
```

> 与室内不同：室外 `$MAP_DIR` 下只会有 `record/`（以及后续生成的 `map_bin/`、配置文件、瓦片），
> **不需要** `global.pcd` / `6D-Pose.txt` / `slam_pose_result.bin` 这些 SLAM 产物。

---

## 5. 启动录制（含驾驶要求）

```bash
/opt/apollo/neo/bin/cyber_recorder record -o "$MAP_DIR/record/map.record" \
  -c /apollo/sensor/lslidar16v4/PointCloud2 \
  -c /apollo/localization/pose \
  -c /apollo/sensor/gnss/imu -c /apollo/sensor/gnss/corrected_imu \
  -c /apollo/sensor/gnss/odometry -c /apollo/sensor/gnss/best_pose \
  -c /apollo/sensor/gnss/ins_stat -c /tf -c /tf_static
echo "MAP_DIR=$MAP_DIR"
```

**9 个通道**。其中 **`/apollo/localization/pose` 是室外的"位姿来源"，缺了这张图就废了**
（室内不需要它，室内用的是 `liorf/mapping/odometry`）。

### 驾驶要求（本工程实测总结，**当前最大的短板**）

| 要求 | 数值 | 为什么 |
| --- | --- | --- |
| **时长** | **连续 5 分钟以上**（至少 3 分钟） | **实测 36 秒 / 53 秒的录像做出的底图完全不可用**（只覆盖十几米） |
| 速度 | 低速平稳，**>0.5 m/s**（1.8 km/h） | CGI-230 用低速车载模式时，动态对准要求 >0.5 m/s |
| **固定解占比** | **>90%** 最好，最低 >80% | 掉解期间位姿精度不足 → 底图重影/扭曲 |
| 场地 | **开阔无遮挡**（远离楼体、树冠、车棚、金属物） | 遮挡是掉解的头号原因 |
| 路线 | 走 1~2 个**闭环**，覆盖**所有要导航的道路** | 覆盖不全 → 后面没底图画车道 |
| 掉解处理 | **停下等恢复**，不要在浮点解下继续跑 | 那段数据不可信 |
| 结束 | `Ctrl-C`（**不要 kill**，否则索引可能不完整） | — |

**建议同时开一个固定解监视**：后台读 `best_pose` + `localization/pose`，
状态变化即打印、每 30 秒报一次车速。**掉解时间点要记下来**，生成底图时据此判断这段数据能否用。
（本工程临时脚本放在容器 `/tmp/fix_watch2.py`，非工作区文件。）

> ⚠️ 实测现象：静止时固定率可达 97~98%，**车一动就频繁掉浮点**（60~98%）。
> 该现象与"安装参数是否正确"无关（回退到原值同样如此），可能与未完成组合导航对准、
> 天线接线方式、或场地多路径有关 —— **尚未定论**（见第 12 节）。

---

## 6. 生成 MapCreator 底图（RTK 模式）

### 6.1 生成配置（C-4 脚本，`--mode rtk`）

```bash
cd /apollo_workspace
python3 modules/tools/yunle_dreamview_bridge/yunle_make_map_creator_image_conf.py \
  --map-dir "$MAP_DIR" --map-name "$MAP_NAME" --mode rtk
```

生成的 `$MAP_DIR/image_creator_conf.pb.txt` 关键项：

```text
reader_conf {
  point_cloud_channel: "/apollo/sensor/lslidar16v4/PointCloud2"
  localization_channel: "/apollo/localization/pose"     # ← 室外位姿来源
}
slam_mode_selection_conf {
  enable_slam_mode: false                               # ← 室外关键：false = RTK 模式
  # slam_pose_path 不参与（室内 SLAM 模式才读 slam_pose_result.bin）
}
```

- `enable_slam_mode: false` → 按 `localization_channel` 与点云**最近时间配对抽帧**
- RTK 模式下点云要变换到 **`localization`** 帧（室内 SLAM 模式用 `imu` 帧）
  → 因此 **`StaticTf` 必须保持运行**，且外参必须是 `localization → lslidar16v4`

### 6.2 跑瓦片生成

```bash
OUT=/apollo_workspace/modules/map_creator/map_editor/data/base_map/$MAP_NAME/map_images
mkdir -p "$OUT"

# ⚠️ 必须以 root 身份执行；以 yunle(1000) 跑会失败且几乎无提示
/opt/apollo/neo/bin/tile_map_images_creator \
  -c "$MAP_DIR/image_creator_conf.pb.txt" \
  -i "$MAP_DIR/record" \
  -o "$OUT"

chown -R 1000:1000 "$OUT" "$MAP_DIR/map_bin"
```

成功标志：

```text
images create succeeded, output dir: .../base_map/<MAP_NAME>/map_images
```

### 6.3 本工程的三个坑（室内不会遇到）

| 坑 | 现象 | 处理 |
| --- | --- | --- |
| **TF 竞态** | `BufferCore::canTransform failed: target_frame localization does not exist` | `StaticTf` 只在 `Init()` 里广播**一次**静态变换，晚启动的生成器可能没收到 → **重试 2~3 次**；仍失败就在 Dreamview 里把 `StaticTf` 关一下再开 |
| **权限** | 以 `yunle` 身份跑失败（输出目录 / `map_bin` 是 root 所有） | 以 **root** 跑，事后 `chown -R 1000:1000` |
| **目录层级** | 底图在 map_creator 里看不到 | 必须是 `<底图名>/map_images/{0..4, tiles.json}`；`-o` 参数**本身就是 `map_images` 目录** |

> 底图列表是后端**每次请求实时扫描** `data/base_map/` 得到的 → 生成完**刷新页面**即可看到，不用重启后端。

### 6.4 底图观感调参（室外几乎必调，室内不需要）

室外点云以**路面**为主，模板默认参数（`k=0.35, x0=15`）会把路面顶成**纯白**，**无法画车道**。
本工程实测（同一块瓦片）：

| 参数 | 作用 | 实测建议 |
| --- | --- | --- |
| `intensity_sigmoid_x0` | **越大越暗** | 15(亮,中位189) → **25(中,中位156)** → 40(暗,中位99) → 70(只留高反射,中位17) |
| `intensity_sigmoid_k` | 越大对比越强 | 0.05 ~ 0.10 |
| `lower/upper_height_limit_relative_to_pose` | 只保留地面薄层 | 例如 `[-0.65, -0.15]`（按实测地面高度确定） |
| `matrix_resolution` | 格子尺寸 | 0.03125(3 cm，噪点多) → **0.05 ~ 0.10**（更干净） |
| `upper_distance_limit` | 保留距离 | 20 → 10~15 m（减少远处杂波） |

> 调参后**必须清空输出目录再重跑**，否则新旧瓦片会混在一起。
> 实测结论：**"一片花白"是渲染参数问题，可调**；但**覆盖不足和位姿精度问题调参救不回来**。

---

## 7. 打开 map_creator 画 HD Map

```bash
# 容器内执行（注意：必须在容器内）
bash /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_start_map_creator_backend.sh
# 浏览器 http://localhost:58000
```

> 手动方式（等价）：`cd /apollo_workspace/modules/map_creator/map_editor && source ~/.nvm/nvm.sh && nvm use 22 && npm run start:backend`

网页操作与室内**完全一致**：

1. 「文件 / 左上菜单」→「打开底图」→ 选 `$MAP_NAME`
2. 画元素：**车道（主导航路线必须画）**、路口、停车位；红绿灯 / 路沿看规划需要
3. 选中车道 → 右侧属性：方向（直行/左转/右转/掉头）、相对方向（单向/反向/双向）、
   类型（机动车道/混合车道等）、**限速**（室外先保守设小，例如 1~3 km/h，第二阶段再提速）
   - 车道**先画宽一些**，避免 `lane width < half vehicle width` 告警
4. 快捷键：左键落点/选中、`Enter`/`Esc` 结束绘制、`Delete` 删除、
   `Ctrl+Z`/`Ctrl+Shift+Z` 撤销/重做、`Shift+点击` 多选
5. 中途点「**保存**」（保存 EditorMap JSON）；画完点「**发布**」（生成 Apollo HDMap）

---

## 8. 发布后安装给 Apollo

发布产物：

```text
/apollo_workspace/modules/map_creator/map_editor/data/released_map/<发布名>/
```

安装（**先 dry-run** 看要动哪些文件）：

```bash
RELEASED_MAP=/apollo_workspace/modules/map_creator/map_editor/data/released_map/<发布名>
python3 /apollo_workspace/modules/tools/yunle_dreamview_bridge/yunle_install_released_hdmap_float_safe.py \
  --src "$RELEASED_MAP" --dry-run
# 确认后去掉 --dry-run 再执行
aem bootstrap restart --plus      # 换地图后必须重启 Dreamview 才能看到新 HDMap
```

安装后目录里应至少有：`base_map.bin/txt`、`sim_map.bin/txt`、`routing_map.bin/txt`、`editor_map.json`。

> ⚠️ **UTM 大坐标的 float 精度问题（室外独有）**：UTM 的 y ≈ 4.43e6，float32 有效精度约 0.5 m。
> 若 Dreamview 里地图完全不显示或明显抖动，需要**统一平移**方案（定位与地图必须同一平移）——
> 这正是 `..._float_safe.py` 脚本名的由来。**室外首次安装前要先确认是否沿用室内的默认平移量。**

---

## 9. Dreamview 验证（定位 + 地图 + 规划）

**第一阶段（推荐）：只验证，不开控制**

依次启动：`LslidarC16V4`、`StaticTf`、`Gnss`、`RtkLocalization`、Routing、Command、Prediction、Planning
—— **先不要启动 `Control` 和 `YunleApolloControlTest`**。

检查：地图能加载 → 「Send Routing」出 route → Planning 出轨迹 → **车辆静止不动** ✔

**第二阶段：低速闭环控制**

确认场地清空、遥控急停可用后，再启动 `YunleApolloControlTest` + `Control`，
Dreamview 切 `Auto_Drive`，按 `Start`。

现有安全边界（要成组评估后才能放开）：命令速度硬上限 **1.5 km/h**、转向硬上限 90%、
命令超时 200 ms、必须 `START` 才转发 `0x121`、终点释放保护。

> 注意：CGI-230 若用"单天线 + 低速车载"模式，其对准确要求 >0.5 m/s，与 1.5 km/h 上限冲突
> —— 这是建议用**双天线模式**的原因之一（双天线不需要该速度条件）。

---

## 10. 常见问题排查 + 安全红线

| 现象 | 首要排查方向 |
| --- | --- |
| `Gnss` 启动即失败 | `gnss_conf.pb.txt` 是否还是 NovAtel；`/dev/serial/by-id/` 路径是否变化 |
| `/apollo/sensor/gnss/raw_data` 无数据 | 串口被调试工具占用；波特率不是 460800；设备输出二进制而非 `$GPCHC` |
| `best_pose` 不是固定解 | 差分未打通（看 `Age`）、遮挡、未完成 INS 对准、`Status` 不是 42 |
| `/apollo/localization/pose` 无输出 | `ins_stat` 缓冲空；非固定解；`gps_imu_time_diff_threshold`(0.02 s) 超限；IMU 不是 100 Hz |
| 定位有但航向差 90°/180° | 硬编码 −90° 假设；设备侧 `headingoffset`/`insangle`；先用静止对准验证 |
| 地图完全不显示 / 抖动 | UTM 大坐标 float 精度 → 统一平移；Dreamview 选的地图名是否正确 |
| 地图加载但车不在车道内 | 底图与定位是否同系；C-2 外参；安装时是否被平移过 |
| **底图一片白** | 强度映射过曝 → 调 `x0`/`k`（见 6.4） |
| **底图重影 / 扭曲** | 录制期间固定解占比不足（<80%）；外参不对；时间不同步 |
| **底图几乎没有内容** | **录制时间太短/路线覆盖不够**（本工程当前的主要问题） |
| `tile_map_images_creator` 报静态 TF 失败 | `StaticTf` 未运行；`localization → lslidar16v4` 外参缺失；重试 |
| Dreamview 模块开关点不动 | `monitor is not turned on` → 见第 3 节"启动顺序" |
| Control 拿到跳变的加速度/角速度 | **两个 IMU 驱动同时运行**（N100 + CGI-230 写同一话题） |

### 互斥与安全红线（沿用现有约定）

- 同一时刻**只能有一个**发布 `/apollo/localization/pose` 的组件（NDT 各入口、RTK 定位、PointCloudBridge 互斥）
- `IndoorTestTf` **不能**与 `StaticTf` 同时启动
- **N100 驱动不能与 CGI-230 的 GNSS 驱动同时启动**（同话题双发布者）
- 三个 Yunle receiver 共用 UDP `8002`，只能选一个
- `YunleApolloControlTest` 会发送 `0x121`；**只观察时应使用 `YunleChassisPreview`**

---

## 11. 室内 vs 室外 差异对照（总表）

| 维度 | 室内 | 室外（RTK） |
| --- | --- | --- |
| **位姿来源** | SLAM（激光 + IMU 里程计） | **CGI-230 的 RTK/INS 解算** |
| **坐标系** | SLAM 局部系（起点为原点） | **UTM / WGS84 绝对坐标** |
| Dreamview 模式 | `Yunle Indoor Mapping Tests` | **`Yunle Outdoor Mapping`** |
| 采图模块 | LslidarC16V4、N100Imu、IndoorTestTf、IndoorSlamMapping | **LslidarC16V4、StaticTf、Gnss、RtkLocalization、YunleChassisPreview** |
| **录制通道** | 5 个（点云、imu、corrected_imu、`/tf`、`liorf/mapping/odometry`） | **9 个**（点云、**`/apollo/localization/pose`**、imu、corrected_imu、odometry、best_pose、ins_stat、`/tf`、`/tf_static`） |
| 对外部环境依赖 | 室内结构化环境即可 | **必须差分通畅 + 开阔天空**（掉解即废） |
| **闭环** | **需要**（SLAM 回环消漂移） | **不需要**（绝对定位） |
| 速度要求 | 慢速慢走即可 | **>0.5 m/s**（否则设备无法完成动态对准） |
| **关键质量指标** | 回环是否闭合、点云糊不糊 | **固定解占比 >90%**、`sol_type = INS_RTKFIXED` |
| **中间产物** | `global.pcd`、`6D-Pose.txt`、`slam_pose_result.bin`、`gnss-map-offset.txt` | **无**（只需 `record/`） |
| **底图配置** | `enable_slam_mode: true` + `slam_pose_path` | **`enable_slam_mode: false`**（走 `localization_channel`） |
| 点云变换目标帧 | `imu` | **`localization`**（依赖 `StaticTf` 外参） |
| **NDT 定位地图** | **必须要**（`yunle_prepare_and_install_indoor_ndt_slot.py` 生成并装固定槽位） | **不需要**（定位直接来自 RTK） |
| 定位器 | PCL-OMP NDT（硬编码加载 `global.pcd`） | `rtk_localization`（纯 RTK） |
| **建图起点约束** | **NDT 初始位置必须在地图起点**（绿点） | **无此约束** |
| 安装脚本 | `yunle_prepare_and_install_indoor_ndt_slot.py` | **`yunle_install_released_hdmap_float_safe.py`**（只装 HDMap） |
| 速限 | 室内低速（1 km/h） | 可更高（第一阶段仍建议保守） |
| **坐标精度风险** | 无 | **UTM y≈4.43e6，float32 精度仅 ~0.5 m** → 需统一平移 |
| **最大不确定性** | SLAM 漂移 | **设备安装参数 / 航向 / 固定解质量** |

---

## 12. 当前状态与待完成事项（**未跑通**）

### 12.1 已验证通过的部分 ✔

| 项 | 证据 |
| --- | --- |
| 设备安装参数（九项）已定稿并持久化 | `getting gilccfg` 回读一致；断电复验通过 |
| 航向正确 | 车头朝东（指南针 90°）时设备报 88.76°，误差 1.2° |
| 位置正确 | Apollo UTM 与独立 UTM 实现比对：ΔE 0.19 m / ΔN 0.03 m / Δz 0.00 m |
| 数据链路完整 | 点云 ~10 Hz、`best_pose` ~1 Hz、`localization/pose` ~97 Hz、StaticTf 广播正常 |
| 录制可用 | 9 通道齐全、`is_complete: true`（含 `/apollo/localization/pose`） |
| RTK 模式底图生成 | `--mode rtk` 配置正确，瓦片生成成功，map_creator 能打开显示 |
| 底图渲染调参 | 已能把"一片白"调成中灰（中位 156） |

### 12.2 尚未完成 ✗（本文档因此标注"未跑通"）

| # | 缺口 | 现状 | 下一步 |
| --- | --- | --- | --- |
| 1 | **录制时长/覆盖不足** | 两段实测只有 36 s / 53 s（约 30~40 m），底图只有一个十几米的小团 | 连续录 5 分钟以上，走 1~2 个闭环覆盖完整路线 |
| 2 | **移动时固定解不稳** | 静止 97~98%，移动 60~98%（"一动就掉"） | 先完成组合导航对准；必要时试对调 Fakra 接线 |
| 3 | **组合导航对准未完成** | `Status` 始终 `40`（低半字节 0 = 初始化），未到 `42` | 待做"决定性实验"：把参数临时回退到原值，用达标驾驶再跑一次（速度>0.5 m/s、固定率>80%、约 5 分钟）——若仍不进 42，则试对调两根 Fakra 线 |
| 4 | **未画过室外车道 / 未发布 / 未安装** | `map_creator` 只在室内跑通过 | 等底图可用后按第 7~9 节执行 |
| 5 | **未验证室外导航** | 室内导航已验证；室外的 Routing/Planning 配置仍绑定室内地图与 0.5 m/s 上限 | 见 `YUNLE_OUTDOOR_RTK_PLAN.md` §8（需单独确认的改动） |
| 6 | `global.pcd` / 点云地图 | **未实现**（RTK 模式不跑 SLAM） | 见第 13 节（可选路线） |
| 7 | NDT/MSF 兜底定位 | **未实现** | 见第 13 节（可选路线） |

### 12.3 已排除的怀疑 ✗（省得重复排查）

- **不是"渲染参数导致不可用"**：花白可调（见 6.4），但覆盖与精度问题调参救不回来
- **不是"驾驶速度不够"**：2026-10-13 有一轮实测速度 0.56~1.39 m/s、固定率 87.4%、约 4.5 分钟，**仍未进 42**
- **不是"回退参数就能进 42"**：曾回退到原值，设备同样停在 40（但当时**没有配合达标驾驶**，该实验仍待正式做一次）
- **不是"Dreamview 坏了"**：模块可手动用 `mainboard` 启动，数据链路与录制不受影响

---

## 13. 关于 `global.pcd` 与点云地图（可选路线，**当前未实现**）

### 13.1 室外能不能做？

**能，但不是"一条命令就有"**：

| 路径 | 命令 | 前提 / 障碍 |
| --- | --- | --- |
| ① MSF 链路 | `scripts/msf_simple_map_creator.sh <record> <外参yaml> <zone_id> <输出> <lidar类型>` | 内部为 `cyber_record_parser → poses_interpolator → lossless_map_creator`，**可直接从 record 出点云图**；⚠️ 但需要 `/apollo/sensor/<lidar>/compensator/PointCloud2`（**运动补偿**），而当前 `lslidar.dag` **没有 compensator 组件** |
| ② NDT 建图 | `ndt_map_creator --pcd_folders ... --pose_files ... --zone_id 50 --resolution 0.25 ...` | 需要先有 **pcd 分片 + pose 文件**（多一步转换） |
| ③ 自研转换 | 读 record + `/apollo/localization/pose` → 变换累积 → 写 pcd | 需要**新写脚本**（新增工作区文件，需单独授权） |

> 注意：室内的 `global.pcd` 是 **SLAM 流水线**的产物（靠 `liorf/mapping/odometry`），
> 现成的 `yunle_indoor_map_save_test.py` **依赖 SLAM 模块在跑**，室外 RTK 模式**不能直接用**。

### 13.2 要不要做？—— **不是当前瓶颈** ✗

**画车道不需要 pcd**：map_creator 用的是**二维底图（瓦片）**。
pcd 只在两个场景有价值：

1. **直观判断数据质量**（覆盖范围、重影、路面清晰度 —— 比翻一堆瓦片直观）
2. **将来做点云定位兜底**（RTK 失效时用 NDT/MSF 定位，见 `YUNLE_OUTDOOR_RTK_PLAN.md` §8.3）

**结论：先把"录够 + 固定解稳"解决掉，再考虑 pcd。**

---

## 附：与室内文档的对应关系

| 室内文档章节 | 本文档对应 | 差异 |
| --- | --- | --- |
| 1. 第一次准备 map_creator | §1 | 相同；**强调必须容器内** |
| 2. 录一段新地图 | §3 + §4 + §5 | 模块不同、通道多 4 个、**驾驶要求严格得多** |
| 3. record 目录准备 | §4 | 相同 |
| 4. 启动 record | §5 | 通道不同（多 `/apollo/localization/pose` 等） |
| 5. 保存 SLAM 地图 | **无对应** | **室外不需要** |
| 6. 生成 NDT 定位地图 | **无对应** | **室外不需要**（定位来自 RTK） |
| 7. 生成 map_creator 底图 | §6 | 配置 `enable_slam_mode` 相反；**多了调参坑与权限/TF 坑** |
| 8. 打开 map_creator 编辑 | §7 | **完全相同** |
| 9. 发布后复制给 Apollo | §8 | 安装脚本不同；**多了 UTM 平移问题** |
| 10. 安装地图 | §8 | 同上 |
| 11. 导航 | §9 | 室外 Routing/Planning 配置待定 |
| 新增 | §2 设备检查 / §12 状态 / §13 pcd | **室外独有** |
