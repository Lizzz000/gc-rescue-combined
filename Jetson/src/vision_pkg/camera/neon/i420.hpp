// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 全范围 I420 -> BGR（标量 + AArch64 NEON），支持紧排图和独立行跨度的三个平面。
#pragma once

#include <opencv2/core.hpp>

namespace vision_camera {
cv::Mat fullRangeI420ToBgr(const cv::Mat & image);
cv::Mat fullRangeI420ToBgr(const cv::Mat & luma, const cv::Mat & cb, const cv::Mat & cr);
}
