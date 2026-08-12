# Map Editor 中文说明

`map_editor` 是一个可移植的本地地图编辑项目，统一包含前端和后端：

- `frontend/`：基于 React、TypeScript、Three.js 的地图编辑器前端。
- `backend/`：基于 Node.js、Express、WebSocket 的后端服务，用于提供底图瓦片、读写 EditorMap JSON，并调用地图转换工具发布地图。

该项目由 `map_editor_frontend_v2` 和 `simple_map_backend_v2` 汇总迁移而来。迁移过程中保留了原有前端布局和地图编辑功能，只增加了跨平台配置、统一构建脚本和部署说明。

## 功能概览

- 从可配置的数据目录加载底图 `tiles.json` 和瓦片图片。
- 支持编辑车道、边界、路口、人行横道、减速带、停车位、停止线、交通灯、标志牌、区域、道闸等地图元素。
- 通过后端 WebSocket 接口保存 EditorMap JSON。
- 通过外部 `editor_map_converter` 工具发布高精地图产物。
- 开发环境可分别启动前端和后端。
- 生产环境可由后端直接托管构建后的前端页面。
- 端口、数据路径、转换器路径均可通过配置文件或环境变量修改，不需要改源码。

## 目录结构

```text
map_editor/
  backend/                 # Node.js 后端服务
  frontend/                # React/Three.js 前端
  data/                    # 默认运行数据目录
    base_map/              # 底图瓦片目录
    editor_map/            # 保存的 EditorMap JSON
    released_map/          # 发布后的地图产物
  package.json             # 根目录统一脚本
  README.md                # 英文说明
  README_CN.md             # 中文说明
```

## 环境要求

必需：

- Node.js `>=20`
- npm

已在当前工作区验证：

- Node.js `20.19.5`，npm `10.8.2`
- Node.js `22.22.2`，npm `10.9.7`

可选：

- nvm 或 nvm-windows，用于切换 Node.js 版本。
- Bazel 和 `map_tool`，仅当需要执行“发布地图”并生成 Apollo HDMap 产物时需要。

支持系统：

- Windows
- Ubuntu / 其他 Linux 发行版

## 安装依赖

在项目根目录执行：

```bash
cd map_editor
npm run ci:all
```

如果是本地开发，不要求严格按锁文件安装，也可以执行：

```bash
npm run install:all
```

Windows PowerShell 如果因为执行策略拦截 `npm.ps1`，请使用：

```powershell
cd D:\workspace\private_tools\map_editor
npm.cmd run ci:all
```

## 后端配置

后端主配置文件：

```text
backend/server.config.json
```

默认内容：

```json
{
  "port": 58000,
  "baseMapRoot": "../data/base_map",
  "editorMapRoot": "../data/editor_map",
  "releaseRoot": "../data/released_map",
  "converterBinary": "/opt/apollo/neo/bin/editor_map_converter",
  "vehicleConfigPath": "/apollo/modules/common/data/vehicle_param.pb.txt",
  "skipValidation": false,
  "serveFrontend": true,
  "frontendBuildDir": "../frontend/build/map_editor_frontend"
}
```

说明：

- `port`：后端 HTTP 和 WebSocket 端口。
- `baseMapRoot`：底图根目录。
- `editorMapRoot`：EditorMap JSON 保存目录。
- `releaseRoot`：发布产物输出目录。
- `converterBinary`：地图转换工具路径。
- `vehicleConfigPath`：Apollo 车辆参数文件路径，发布时会作为 `--vehicle_config_path` 传给转换器。
- `skipValidation`：发布时是否跳过校验。
- `serveFrontend`：是否由后端托管构建后的前端。
- `frontendBuildDir`：前端生产构建目录。

相对路径会基于 `backend/server.config.json` 所在目录解析，因此默认 `../data/base_map` 在 Windows 和 Ubuntu 上都会指向：

```text
map_editor/data/base_map
```

也可以通过环境变量指定另一个配置文件：

```bash
MAP_EDITOR_CONFIG=/absolute/path/to/server.config.json npm run start:backend
```

## 后端环境变量覆盖

以下环境变量会覆盖 `server.config.json`：

| 环境变量 | 说明 |
|---|---|
| `MAP_EDITOR_PORT` 或 `PORT` | 后端端口 |
| `MAP_EDITOR_BASE_MAP_ROOT` | 底图根目录 |
| `MAP_EDITOR_EDITOR_MAP_ROOT` | EditorMap JSON 目录 |
| `MAP_EDITOR_RELEASE_ROOT` | 发布产物目录 |
| `MAP_EDITOR_CONVERTER_BINARY` | `editor_map_converter` 路径 |
| `MAP_EDITOR_VEHICLE_CONFIG_PATH` | `vehicle_param.pb.txt` 路径 |
| `MAP_EDITOR_SKIP_VALIDATION` | 是否跳过校验，`true` / `false` |
| `MAP_EDITOR_SERVE_FRONTEND` | 是否托管前端，`true` / `false` |
| `MAP_EDITOR_FRONTEND_BUILD_DIR` | 前端构建目录 |

Windows PowerShell 示例：

```powershell
$env:MAP_EDITOR_PORT = "58100"
$env:MAP_EDITOR_BASE_MAP_ROOT = "D:\maps\base_map"
npm.cmd run start:backend
```

Ubuntu 示例：

```bash
MAP_EDITOR_PORT=58100 MAP_EDITOR_BASE_MAP_ROOT=/data/maps/base_map npm run start:backend
```

## 前端后端地址配置

前端运行时配置文件：

```text
frontend/public/map-editor.config.js
```

生产构建后会复制到：

```text
frontend/build/map_editor_frontend/map-editor.config.js
```

部署后可以直接修改构建产物中的该文件，无需重新构建前端：

```js
window.MAP_EDITOR_CONFIG = {
  backendHost: "127.0.0.1:58000",
  backendPort: "58000"
};
```

注意：

- `backendHost` 不需要写 `http://` 或 `ws://`。
- 如果 `backendHost` 为空，开发环境默认使用当前浏览器 hostname 加 `backendPort`。
- 生产环境默认使用 `window.location.host`，适合由后端同端口托管前端页面。

也支持构建时变量：

```bash
REACT_APP_MAP_BACKEND_HOST=127.0.0.1:58000 npm run build
REACT_APP_MAP_BACKEND_PORT=58000 npm run build
```

## 开发环境运行

启动后端：

```bash
cd map_editor
npm run start:backend
```

另开终端启动前端：

```bash
cd map_editor
npm run start:frontend
```

访问：

```text
http://127.0.0.1:3000
```

默认情况下，前端会连接：

```text
ws://127.0.0.1:58000/plugins/map
http://127.0.0.1:58000/mapcreator/...
```

如果 `58000` 端口被占用，请修改后端 `MAP_EDITOR_PORT`，并同步调整前端 `map-editor.config.js` 或 `REACT_APP_MAP_BACKEND_PORT`。

## 生产构建与运行

构建前端：

```bash
cd map_editor
npm run build
```

构建输出目录：

```text
frontend/build/map_editor_frontend
```

启动后端：

```bash
npm run start:backend
```

当 `serveFrontend=true` 且 `frontendBuildDir` 指向有效构建目录时，后端会同时提供：

```text
http://127.0.0.1:58000/          # 前端页面
http://127.0.0.1:58000/healthz   # 健康检查
ws://127.0.0.1:58000/plugins/map # WebSocket 接口
```

## 数据目录准备

默认数据根目录：

```text
map_editor/data
```

底图目录需要满足：

```text
data/base_map/<mapName>/map_images/tiles.json
data/base_map/<mapName>/map_images/<level>/<y>/<tile-file>
```

后端只会列出包含以下文件的底图目录：

```text
map_images/tiles.json
```

保存的 EditorMap 文件位置：

```text
data/editor_map/<mapName>.json
```

发布产物输出位置：

```text
data/released_map/<mapName>/
```

## 发布地图

发布流程由前端触发 `ReleaseMapFile` WebSocket 请求，后端会先保存 EditorMap JSON，然后调用：

```text
converterBinary
```

默认转换器路径：

```text
bazel-bin/modules/private_tools/map_tool/editor_map_converter
```

Windows 下如果配置路径没有扩展名，且原路径不存在，后端会自动尝试同路径加 `.exe`。

如果需要构建转换器，可在对应 Apollo/Bazel 工作区执行：

```bash
bazel build //modules/private_tools/map_tool:editor_map_converter
```

如果转换器在其他位置，请修改 `backend/server.config.json` 中的 `converterBinary`，或使用环境变量：

```bash
MAP_EDITOR_CONVERTER_BINARY=/absolute/path/to/editor_map_converter npm run start:backend
```

## 常用验证命令

检查前端 TypeScript 和后端语法：

```bash
npm run check
```

构建前端：

```bash
npm run build
```

健康检查：

```bash
curl http://127.0.0.1:58000/healthz
```

期望返回：

```json
{"status":"ok"}
```

## 已验证结果

当前工作区已验证：

- Node.js `20.19.5`
  - `npm run ci:backend` 通过
  - `npm run ci:frontend` 通过
  - `npm run check` 通过
  - `npm run build` 通过
  - 后端托管生产前端验证通过：`/healthz` 和 `/` 均返回 HTTP 200
- Node.js `22.22.2`
  - `npm run check` 通过
  - `npm run build` 通过

构建时仍会出现原前端项目已有的 warning，例如 React Hook dependency、`no-param-reassign`、bundle size 和 Browserslist 数据过期提示。这些 warning 不阻塞编译和运行。

## 干净打包建议

建议迁移或部署时不要携带 `node_modules`。

推荐保留内容：

```text
map_editor/
  backend/
  frontend/
  data/
  package.json
  README.md
  README_CN.md
  .gitignore
  .nvmrc
```

拷贝到新机器后执行：

```bash
cd map_editor
npm run ci:all
npm run build
npm run start:backend
```
