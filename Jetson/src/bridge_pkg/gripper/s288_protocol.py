"""薄转发：S288 协议只在 teleop_pkg 的 servo.s288，本模块不重复实现。

导入依赖已安装的 servo 包（source 工作区 setup.bash 后进入 PYTHONPATH）。
"""

from servo.s288 import (  # noqa: F401
    BAUD,
    COMMAND_PERIOD_S,
    CONTROL_MARK,
    FEEDBACK_BYTES,
    FEEDBACK_MARK,
    FEEDBACK_WAIT_S,
    MODE_RUN,
    MODE_STOP,
    RATIO,
    STOP_ATTEMPTS,
    _POSITION_SCALE,
    control_packet,
    crc32,
    parse_feedback,
)
