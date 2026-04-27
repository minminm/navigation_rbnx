#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
FLAGS+=(--mcp)

if command -v rbnx &>/dev/null; then
  rbnx codegen -p "$PKG" "${FLAGS[@]}"
fi

docker compose -f "$PKG/docker/compose.yaml" build

echo "[build] done."
