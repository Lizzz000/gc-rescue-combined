#pragma once

// 生产的 interface_pkg/app/core.hpp 会再包含 ros/ros.hpp。
// host 测试把本文件放在更靠前的 -I 上，只打开纯值头，这样不必改生产代码，也不必安装 ROS。
#include <interface_pkg/app/core/core.hpp>
