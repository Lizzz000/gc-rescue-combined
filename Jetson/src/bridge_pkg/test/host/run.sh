#!/usr/bin/env bash
# 本机/板端 g++ 编译 bridge_pkg 纯核心并跑行为基线（与 run.ps1 相同的源文件与桩）。
# 产物放在 ${TMPDIR:-/tmp}/gc_rescue_host/bridge_pkg。
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
package="$(cd "$here/../.." && pwd)"
src="$(cd "$package/.." && pwd)"
out="${TMPDIR:-/tmp}/gc_rescue_host/bridge_pkg"
mkdir -p "$out"
${CXX:-g++} -std=c++17 -Wall -Wextra -I"$here" -I"$here/stubs" -I"$package" -I"$src" \
  "$here/test_main.cpp" "$here/test_can_protocol.cpp" "$here/test_tx_queue.cpp" \
  "$here/test_bridge_state.cpp" "$package/state/bridge_state.cpp" -o "$out/bridge_host_tests"
"$out/bridge_host_tests"
