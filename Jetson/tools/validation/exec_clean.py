#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""只在验收器自己创建的子进程中恢复信号，再替换为真正命令。"""
import os
import signal
import sys

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    os.execvpe(sys.argv[1], sys.argv[1:], os.environ)
