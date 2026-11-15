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
#include <iostream>
// prepare Stop + last motion exchange + final Stop + final period + link/retry margin.
constexpr double total_ms = 1000 * (s288::kMaxMoveS +
    (2 * s288::kStopAttempts + 1) * s288::kFeedbackWaitS + s288::kCommandPeriodS) +
    JETSON_AUTO_REQUEST_LINK_MARGIN_MS;
static_assert(JETSON_AUTO_ACTION_TIMEOUT_MS >= total_ms,
              "STM32 action deadline does not cover the full S288 action");
int main() {
    std::cout << "PASS: S288 maximum budget " << total_ms << " ms <= STM32 "
              << JETSON_AUTO_ACTION_TIMEOUT_MS << " ms\\n";
}
''', encoding='utf-8')
        executable = Path(temporary) / 'budget.exe'
        subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-I' + str(root / 'STM32/project/Core/Control/Jetson'),
                        '-I' + str(root / 'Jetson/src/bridge_pkg'),
                        str(source), '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
