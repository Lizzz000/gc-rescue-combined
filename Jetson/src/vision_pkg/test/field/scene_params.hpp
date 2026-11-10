// 安全区测试的全部常量与阈值集中在这里；算法参数（N、滤波系数、帧率上限）一律从 yaml 生成的
// field_config_values.inc 读取，不在此重复。
#pragma once

#include <opencv2/core.hpp>

namespace field_test {

// ---- 序列长度 ----
constexpr int kMatrixFrames = 8;          // 矩阵每例帧数：前几帧留给定位确认（locate_hits_needed）
constexpr int kMatrixScored = 4;          // 矩阵每例只统计最后几帧
constexpr int kMatrixSeeds = 3;           // 普通矩阵每例种子数（含相机位置微扰）
constexpr int kGridSeeds = 1;             // 物块网格例数多，只跑 1 个种子
constexpr double kSeedCamShiftX = 0.03;   // 种子微扰：相机横移，米/档
constexpr double kSeedCamShiftD = 0.04;   // 种子微扰：相机距离，米/档

// ---- 真值：中线可见性（渲染差分 + 像素比例） ----
constexpr double kVisDiffMin = 12.0;      // 有/无中线两图同像素最大通道差 ≥ 该值才算“看得见”，灰度级
constexpr double kJudgeableGeometricMin = 0.30;  // 可判：干净场景可见足迹 / 几何足迹下限（围边遮挡后）
constexpr double kUnjudgeableGeometricMax = 0.15;// 不可判：围边/画框把中线挡到只剩该比例以下
constexpr double kVisSampleStepM = 0.002;      // 中线足迹纵向采样步长，米
constexpr int kVisSamplesAcross = 5;            // 中线足迹横向采样点数
constexpr int kCleanJpegQuality = 100;          // 干净参考图的 JPEG 质量
constexpr double kJudgeableMinRatio = 0.60;     // 可判：中线地面足迹可见比例下限
constexpr double kJudgeableEndMinRatio = 0.50;  // 可判：远/近三分之一段各自可见比例下限
constexpr double kUnjudgeableMaxRatio = 0.30;   // 不可判：总可见比例上限
constexpr double kUnjudgeableEndMaxRatio = 0.15;// 不可判：任一端三分之一段可见比例上限
// 介于两者之间为“边界”：不计有效率，只要求不输出错误分区。

// ---- 判据：抗干扰 ----
constexpr double kMinValidRate = 0.90;        // 单因素正例：可判帧的分区有效率下限
constexpr double kMinValidRateCombo = 0.75;   // 组合正例：可判帧的分区有效率下限
constexpr int kMaxFalseOnUnjudgeable = 0;     // 不可判帧输出有效分区的帧数上限
constexpr int kMaxFalseOnNegative = 0;        // 负例输出有效外框/分区的帧数上限

// ---- 判据：准确 ----
constexpr double kSplitLabelTolPx = 10.0;     // 分区标签正确：中线横向误差上限（处理图 1280 宽）
constexpr double kSplitErrMaxPx = 2.0;        // 中线横向误差均值上限
constexpr double kEdgeLineTolPx = 2.0;        // 四条边到真值内沿线的平均距离上限（画内采样）
constexpr double kEdgeLineTolNearPx = 4.0;    // 近距离视角放宽
constexpr double kCornerTolPx = 3.0;          // 画内角点误差上限（仅统计画内角点）
constexpr int kEdgeSamples = 16;              // 每条边的采样点数
constexpr double kCenterTolPx = 4.0;          // 半区中心误差上限；J6 纯函数落地前只报告不判
constexpr bool kGateCenterError = false;      // J6 落地后改为 true

// ---- 判据：稳定（序列） ----
constexpr int kSequenceFrames = 200;          // 静态序列帧数
constexpr int kWarmupMargin = 3;             // 统计前丢弃 locate_hits_needed（yaml）+ 该余量帧
constexpr double kMaxSplitJitterPx = 1.0;     // 静止时中线横向位置标准差上限
constexpr double kMaxCornerJitterPx = 1.0;    // 静止时画内角点标准差上限（四角均值）
constexpr int kMaxFlipsStatic = 0;            // 静止且无注入时分区有效↔无效翻转次数上限
constexpr int kMaxPredictedAsMeasured = 0;    // 输出有效但边状态为预测/质量为预测保持的帧数上限
constexpr int kInjectEvery = 40;              // 注入间隔帧数
constexpr int kInjectLengths[] = {1, 2};      // 每次注入连续帧数（单帧、两帧）
constexpr double kInjectExposure = 0.05;      // 注入帧曝光：中线差分低于 kVisDiffMin，真值为不可判
constexpr int kMaxRecoverFrames = 1;          // 注入结束后恢复有效所需帧数上限（分区层无保持）
constexpr double kAeRampLow = 0.55;           // 自动曝光爬坡最低增益
constexpr int kAeRampFrames = 60;             // 爬坡半周期帧数
constexpr double kAeJumpGain = 1.45;          // 自动曝光跳变目标增益
constexpr int kAeJumpFrames = 30;            // 跳变保持帧数，之后跳回 1.0
constexpr double kAeReportBins[] = {0.55, 0.70, 0.85, 1.00, 1.50};  // 报告用增益分档边界（左闭右开）
constexpr double kMinValidRateAe = 0.90;      // 爬坡/跳变期间可判帧有效率下限

// ---- 计时（只做本机相对比较；绝对值需板端） ----
constexpr double kTimingRegressionRatio = 1.20; // 中位数不得超过参考值的倍数（参考值来自环境变量）
constexpr const char * kTimingReferenceEnv = "FIELD_TIMING_REFERENCE_MS";

// ---- 回放 ----
constexpr const char * kReplayDirEnv = "FIELD_REPLAY_DIR";
constexpr int kExitSkipped = 77;              // ctest/ament 约定的跳过码

// ---- 生成器颜色（BGR） ----
const cv::Scalar kBlack(28, 28, 30), kWhite(235, 235, 232), kBlue(170, 80, 30), kRed(70, 72, 160),
  kYellow(40, 200, 220), kGreen(60, 160, 60), kPurple(190, 95, 120);
const cv::Scalar kLightGrayDivider(82, 95, 109);   // 真机强光下中线实测颜色
const cv::Vec3d kTintWarm(0.7, 0.95, 1.3), kTintCold(1.3, 1.0, 0.8), kTintGreen(0.9, 1.12, 0.92);
constexpr double kAwbResidual = 0.8;          // 自动白平衡补偿 80%，剩 20% 残差
constexpr double kShadowLight = 0.30, kShadowDark = 0.55;

// ---- 物块网格 ----
constexpr double kBlockSize = 0.05;                        // 方块边长，米
constexpr double kBlockHeights[] = {0.01, 0.02, 0.03, 0.05}; // 1–5 cm
constexpr double kBlockRows[] = {0.08, 0.18, 0.28};        // 近/中/远（场地 y，米）
constexpr double kBlockOffsets[] = {0.0, 0.036, 0.12};     // 压线/贴线（半带宽+半块）/离线（场地 x，米）

}  // namespace field_test
