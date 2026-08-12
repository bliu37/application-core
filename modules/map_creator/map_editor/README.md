# Map Editor

`map_editor` is a portable local map editing project that combines:

- `frontend/`: React + TypeScript + Three.js map editor UI.
- `backend/`: Node.js + Express + WebSocket service for tiles, EditorMap JSON files, and map publishing.

The project is migrated from `map_editor_frontend_v2` and `simple_map_backend_v2` into one deployable directory. The frontend layout and editing features are preserved; only portability, configuration, and build/runtime wiring were adjusted.

## Features

- Load base map tile metadata and tile images from a configurable data directory.
- Edit map elements including lanes, boundaries, junctions, crosswalks, speed bumps, parking spaces, stop lines, traffic signals, signs, areas, and barrier gates.
- Save EditorMap JSON files through the backend WebSocket API.
- Publish maps by invoking an external `editor_map_converter` binary.
- Run the frontend and backend separately for development.
- Build the frontend and let the backend serve the production frontend from one port.
- Configure paths and ports without modifying source code.

## Directory Layout

```text
map_editor/
  backend/                 # Node.js backend service
  frontend/                # React/Three.js frontend
  data/                    # Default runtime data root
    base_map/              # Base map tiles
    editor_map/            # Saved EditorMap JSON files
    released_map/          # Published outputs
  package.json             # Root convenience scripts
  README.md
```

## Runtime Requirements

Required:

- Node.js `>=20`
- npm bundled with Node.js

Tested in this workspace:

- Node.js `20.19.5` with npm `10.8.2`
- Node.js `22.22.2` with npm `10.9.7`

Optional:

- nvm or nvm-windows for switching Node versions.
- Bazel and `map_tool` if you need the publish step to generate Apollo HDMap outputs.

Operating systems:

- Windows
- Ubuntu / other Linux distributions with Node.js `>=20`

## Install Dependencies

From the project root:

```bash
cd map_editor
npm run ci:all
```

If you are iterating locally and do not need strict lockfile installs:

```bash
npm run install:all
```

On Windows PowerShell, if direct `npm` is blocked by execution policy, use `npm.cmd`:

```powershell
cd D:\workspace\private_tools\map_editor
npm.cmd run ci:all
```

## Configuration

### Backend Configuration File

Main backend config:

```text
backend/server.config.json
```

Example:

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

Relative paths in `server.config.json` are resolved relative to the config file directory, so the default `../data/base_map` points to `map_editor/data/base_map` on both Windows and Ubuntu.

When running inside Apollo Docker, `converterBinary` should point to the installed converter binary, usually `/opt/apollo/neo/bin/editor_map_converter`. `vehicleConfigPath` is forwarded as `--vehicle_config_path` so routing map generation can load Apollo vehicle parameters from an absolute path.

You can also point to another config file:

```bash
MAP_EDITOR_CONFIG=/absolute/path/to/server.config.json npm run start:backend
```

### Backend Environment Overrides

Environment variables override `server.config.json`:

| Variable | Description |
|---|---|
| `MAP_EDITOR_PORT` or `PORT` | Backend HTTP/WebSocket port |
| `MAP_EDITOR_BASE_MAP_ROOT` | Base map root directory |
| `MAP_EDITOR_EDITOR_MAP_ROOT` | EditorMap JSON directory |
| `MAP_EDITOR_RELEASE_ROOT` | Published map output directory |
| `MAP_EDITOR_CONVERTER_BINARY` | Path to `editor_map_converter` |
| `MAP_EDITOR_VEHICLE_CONFIG_PATH` | Path to `vehicle_param.pb.txt` passed to converter |
| `MAP_EDITOR_SKIP_VALIDATION` | `true` / `false`, forwarded to converter |
| `MAP_EDITOR_SERVE_FRONTEND` | `true` / `false`, whether backend serves built frontend |
| `MAP_EDITOR_FRONTEND_BUILD_DIR` | Built frontend directory |

Examples:

Windows PowerShell:

```powershell
$env:MAP_EDITOR_PORT = "58100"
$env:MAP_EDITOR_BASE_MAP_ROOT = "D:\maps\base_map"
npm.cmd run start:backend
```

Ubuntu:

```bash
MAP_EDITOR_PORT=58100 MAP_EDITOR_BASE_MAP_ROOT=/data/maps/base_map npm run start:backend
```

### Frontend Runtime Backend Address

The frontend reads runtime config from:

```text
frontend/public/map-editor.config.js
```

This file is copied into the production build as:

```text
frontend/build/map_editor_frontend/map-editor.config.js
```

You can edit the built file after deployment without rebuilding the frontend:

```js
window.MAP_EDITOR_CONFIG = {
  backendHost: "127.0.0.1:58000",
  backendPort: "58000"
};
```

`backendHost` should not include `http://` or `ws://`; the frontend adds those where needed. If `backendHost` is empty, development mode defaults to the browser hostname plus `backendPort`, and production mode defaults to `window.location.host`.

Build-time fallback variables are also supported:

```bash
REACT_APP_MAP_BACKEND_HOST=127.0.0.1:58000 npm run build
REACT_APP_MAP_BACKEND_PORT=58000 npm run build
```

## Development Workflow

Start backend:

```bash
cd map_editor
npm run start:backend
```

Start frontend dev server in another terminal:

```bash
cd map_editor
npm run start:frontend
```

Open:

```text
http://127.0.0.1:3000
```

By default, the frontend connects to:

```text
ws://127.0.0.1:58000/plugins/map
http://127.0.0.1:58000/mapcreator/...
```

If port `58000` is occupied, set `MAP_EDITOR_PORT` for backend and update `frontend/public/map-editor.config.js` or `REACT_APP_MAP_BACKEND_PORT` for frontend.

## Production Build and Run

Build frontend:

```bash
cd map_editor
npm run build
```

Build output:

```text
frontend/build/map_editor_frontend
```

Start backend:

```bash
npm run start:backend
```

When `serveFrontend` is `true` and `frontendBuildDir` points to a valid build, the backend serves both:

```text
http://127.0.0.1:58000/          # frontend page
http://127.0.0.1:58000/healthz   # backend health check
ws://127.0.0.1:58000/plugins/map # WebSocket API
```

## Data Preparation

Default data root:

```text
map_editor/data
```

Expected base map layout:

```text
data/base_map/<mapName>/map_images/tiles.json
data/base_map/<mapName>/map_images/<level>/<y>/<tile-file>
```

The backend lists only base map directories that contain:

```text
map_images/tiles.json
```

Saved EditorMap files are written to:

```text
data/editor_map/<mapName>.json
```

Published maps are written to:

```text
data/released_map/<mapName>/
```

## Publishing Maps

The `ReleaseMapFile` WebSocket request saves the EditorMap JSON and then invokes:

```text
converterBinary
```

Default:

```text
bazel-bin/modules/private_tools/map_tool/editor_map_converter
```

On Windows, if `converterBinary` does not exist and has no extension, the backend also tries the same path with `.exe`.

Build the converter in the Apollo/Bazel workspace when needed:

```bash
bazel build //modules/private_tools/map_tool:editor_map_converter
```

If your converter lives elsewhere, set `converterBinary` in `backend/server.config.json` or use:

```bash
MAP_EDITOR_CONVERTER_BINARY=/absolute/path/to/editor_map_converter npm run start:backend
```

## Verification Commands

Check frontend TypeScript and backend syntax:

```bash
npm run check
```

Build frontend:

```bash
npm run build
```

Health check:

```bash
curl http://127.0.0.1:58000/healthz
```

Expected:

```json
{"status":"ok"}
```

## Verification Performed

In the current workspace:

- `npm run ci:backend` passed on Node.js `20.19.5`.
- `npm run ci:frontend` passed on Node.js `20.19.5`.
- `npm run check` passed on Node.js `20.19.5`.
- `npm run build` passed on Node.js `20.19.5`.
- Backend production serving was verified on port `58100` with:
  - `/healthz` -> HTTP 200
  - `/` -> HTTP 200 and frontend HTML
- `npm run check` passed on Node.js `22.22.2`.
- `npm run build` passed on Node.js `22.22.2`.

Build warnings remain from the original frontend codebase, mainly React Hook dependency warnings, `no-param-reassign`, bundle size warnings, and outdated Browserslist data. They do not block compilation.

## Clean Packaging Notes

The repository is intended to be moved without `node_modules`.

Recommended portable contents:

```text
map_editor/
  backend/
  frontend/
  data/
  package.json
  README.md
  .gitignore
  .nvmrc
```

After copying to another machine:

```bash
cd map_editor
npm run ci:all
npm run build
npm run start:backend
```
