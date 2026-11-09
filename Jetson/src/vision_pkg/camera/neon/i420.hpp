// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 全范围 I420 -> BGR（标量 + AArch64 NEON），由 camera.cpp 的 GStreamer MJPEG 路径使用。
#pragma once

#include <opencv2/core.hpp>

namespace vision_camera {
cv::Mat fullRangeI420ToBgr(const cv::Mat & image);
}
