"""Compile the two production timing contracts together; no ROS or hardware needed."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cxx', help='host C++ compiler')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    fallback = Path('C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/g++.exe')
    compiler = args.cxx or shutil.which('g++') or (str(fallback) if fallback.exists() else None)
    if not compiler:
        parser.error('host g++ is required')
    with tempfile.TemporaryDirectory(prefix='gripper_budget_') as temporary:
        source = Path(temporary) / 'budget.cpp'
        source.write_text('''
#include "jetson_auto_config.h"
#include "gripper/s288_servo.hpp"
#include <cstdio>
// Leave an in-flight hold and Stop, prepare, move/capture, then finish the last exchange and Stop.
constexpr double stop_s = s288::kStopAttempts * s288::kFeedbackWaitS;
constexpr double exchange_and_tick_s = s288::kFeedbackWaitS + s288::kCommandPeriodS;
constexpr double total_ms = 1000 * (3 * stop_s + 2 * exchange_and_tick_s +
    s288::kMaxMoveS + s288::kHoldCaptureS) +
    JETSON_AUTO_REQUEST_LINK_MARGIN_MS;
static_assert(JETSON_AUTO_ACTION_TIMEOUT_MS >= total_ms,
              "STM32 action deadline does not cover the full S288 action");
int main() {
    std::printf("PASS: S288 maximum budget %g ms <= STM32 %u ms\\n",
                total_ms, static_cast<unsigned>(JETSON_AUTO_ACTION_TIMEOUT_MS));
}
''', encoding='utf-8')
        executable = Path(temporary) / 'budget.exe'
        subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(root / 'STM32/project/Core/Control/Jetson'),
                        '-I' + str(root / 'STM32/project/Core/Control/Chassis'),
                        '-I' + str(root / 'STM32/project/Core/Control/Vision'),
                        '-I' + str(root / 'Jetson/src/bridge_pkg'),
                        str(source), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
