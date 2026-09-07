#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
MAP_EDITOR_DIR="${WORKSPACE_ROOT}/modules/map_creator/map_editor"
BACKEND_PORT="${MAP_EDITOR_PORT:-58000}"
HEALTH_URL="http://127.0.0.1:${BACKEND_PORT}/healthz"
BROWSER_URL="http://localhost:${BACKEND_PORT}"

if [[ ! -d "${MAP_EDITOR_DIR}" ]]; then
  echo "错误：找不到 map_editor 目录：${MAP_EDITOR_DIR}" >&2
  exit 1
fi

export MAP_EDITOR_PORT="${BACKEND_PORT}"

if [[ -n "${NVM_DIR:-}" ]]; then
  NVM_DIR="${NVM_DIR}"
else
  NVM_DIR="${HOME}/.nvm"
fi
export NVM_DIR

if [[ ! -s "${NVM_DIR}/nvm.sh" ]]; then
  echo "错误：找不到 nvm：${NVM_DIR}/nvm.sh" >&2
  exit 1
fi

# nvm is a shell function, so it must be sourced in this shell.
# shellcheck disable=SC1090
source "${NVM_DIR}/nvm.sh"
nvm use 22

if ! command -v npm >/dev/null 2>&1; then
  echo "错误：当前 Node.js 环境中找不到 npm。" >&2
  exit 1
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "错误：脚本需要 curl 来检测后端是否启动成功。" >&2
  exit 1
fi

if curl --silent --show-error --fail "${HEALTH_URL}" >/dev/null 2>&1; then
  echo "Map Creator 后端已经在运行。"
  echo "浏览器打开："
  echo "${BROWSER_URL}"
  exit 0
fi

if command -v lsof >/dev/null 2>&1 && \
   lsof -nP -iTCP:"${BACKEND_PORT}" -sTCP:LISTEN -t >/dev/null 2>&1; then
  echo "错误：端口 ${BACKEND_PORT} 已被其他进程占用。" >&2
  exit 1
fi

cd "${MAP_EDITOR_DIR}"
echo "正在启动 Map Creator 后端，目录：${MAP_EDITOR_DIR}"
echo "Node.js：$(node --version)"
echo "端口：${BACKEND_PORT}"

npm run start:backend &
BACKEND_PID=$!

cleanup() {
  if [[ -n "${BACKEND_PID:-}" ]] && kill -0 "${BACKEND_PID}" 2>/dev/null; then
    kill "${BACKEND_PID}" 2>/dev/null || true
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 130' INT TERM

ready=0
for _ in $(seq 1 60); do
  if curl --silent --show-error --fail "${HEALTH_URL}" >/dev/null 2>&1; then
    ready=1
    break
  fi

  if ! kill -0 "${BACKEND_PID}" 2>/dev/null; then
    set +e
    wait "${BACKEND_PID}"
    status=$?
    set -e
    echo "错误：Map Creator 后端启动失败，退出码：${status}" >&2
    exit "${status}"
  fi
  sleep 1
done

if [[ "${ready}" -ne 1 ]]; then
  echo "错误：后端在 60 秒内没有监听 ${BACKEND_PORT}。" >&2
  exit 1
fi

echo "Map Creator 后端启动成功。"
echo "浏览器打开："
echo "${BROWSER_URL}"
echo "按 Ctrl-C 可停止 Map Creator 后端。"

wait "${BACKEND_PID}"
