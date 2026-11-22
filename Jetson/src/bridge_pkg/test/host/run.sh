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
  "$here/test_main.cpp" "$here/test_usb_protocol.cpp" "$here/test_tx_queue.cpp" \
  "$here/test_bridge_state.cpp" "$here/test_arrival_point.cpp" "$here/test_search_request.cpp" \
  "$here/test_bridge_transactions.cpp" "$package/state/bridge_state.cpp" \
  "$package/state/bridge_transactions.cpp" "$package/transport/transport.cpp" -o "$out/bridge_host_tests"
"$out/bridge_host_tests"

${CXX:-g++} -std=c++17 -Wall -Wextra -I"$here/runtime_stubs" -I"$here/stubs" -I"$here" -I"$package" -I"$src" \
  "$here/test_runtime_main.cpp" "$here/test_bridge_runtime.cpp" "$package/node/bridge_io.cpp" \
  "$package/node/bridge_session.cpp" "$package/state/bridge_state.cpp" \
  "$package/state/bridge_transactions.cpp" "$package/transport/transport.cpp" \
  -o "$out/bridge_runtime_tests"
"$out/bridge_runtime_tests"

${CXX:-g++} -std=c++17 -Wall -Wextra -I"$here/posix_stubs" -I"$here/stubs" -I"$here" -I"$package" -I"$src" \
  "$here/test_cdc_write.cpp" "$package/transport/cdc/cdc.cpp" -o "$out/cdc_write_tests"
"$out/cdc_write_tests"

${CXX:-g++} -std=c++17 -Wall -Wextra -pthread -I"$here/posix_stubs" -I"$here" -I"$package" \
  "$here/test_s288_timing.cpp" "$package/gripper/s288_servo.cpp" -o "$out/s288_timing_tests"
"$out/s288_timing_tests"
