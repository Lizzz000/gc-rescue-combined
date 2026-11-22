#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""板端综合验收入口；阶段调度和结果记录由 validation.workflow 负责。

实际运行条件与用法见 docs/README.md 的分层验收说明。
"""
import signal
from validation.workflow import main

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    raise SystemExit(main())
