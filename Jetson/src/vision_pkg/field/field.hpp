#pragma once
// ---- 场地 ----
// 职责：安全区几何、参数和观察结果。
// Copyright 2026 jetson_yolo integration
// SPDX-License-Identifier: Apache-2.0
// 紫框场地定位与 SEARCH/LOCATE/TRACK 状态机，仅依赖 OpenCV。
// 每帧先用紫框与内部颜色证据筛候选，再更新跟踪状态，最后生成鸟瞰与分区诊断：
//   预处理 -> ROI／全图搜索 -> 拟合、评分、准入 -> 保持／跳变确认／提交
//   -> 透视矫正 -> 黑带中线 split_x 与内部颜色占比。正式几何发布另由 ROS 转换边界判定。
// 默认参数针对紫色边框、红色场地；角点顺序固定为图像中的 TL/TR/BR/BL。

#include <opencv2/core.hpp>
#include <string>
#include <vector>

// EdgeState：MEASURED=本帧实测；PREDICTED=用上一帧预测或候选先验补的；LOST=无证据。
enum class EdgeState { MEASURED = 0, PREDICTED = 1, LOST = 2 };

// CandidateScore：单个候选的评分明细；先过各项硬门槛，再用 total 在同类候选中排序。
struct CandidateScore
{
  double edge_support[4] = {0, 0, 0, 0};     // 上、右、下、左四边的紫色支持度
  double purple = 0.0;         // 紫框证据
  double red_inside = 0.0;     // 四边形内所选颜色占比；red_* 名称保持兼容
  double red_outside = 0.0;    // 外侧同色风险: max(环形占比, 单边最大占比/3)，越低越好
  double temporal = 0.0;       // 与上一帧预测的一致性 (1 = 完全一致)
  double geometry = 0.0;       // 形状合理性
  double total = 0.0;          // 加权总分
  double top2_min = 0.0;       // 第二强边的支持度，避免仅一条强边支撑候选
  int cells_red = 0;           // 内部网格中符合所选颜色的格数；网格默认 4×2，可由配置改变
  int edges_ok = 0;            // 支持度 >= cfg.edge_support_min 的边数
  bool crosses_interior = false; // 无紫边支持的区段内，两侧同为内部颜色的反证过多，禁止当外框提交
};

// 纯值场地算法参数；使用前统一检查尺寸、比例和门槛范围。
struct FieldConfig
{
  // 新参数同时登记节点字段映射、统一配置及 validateFieldConfig；核心默认值仅供直接开发 API 使用。
  // ---- [分组] 处理尺寸：宽为正像素数；仅高度为 0 时按源图宽高比推算 ----
  int proc_width = 640;
  int proc_height = 0;                   // 0 = 按原图比例

  // 场地真实尺寸与鸟瞰分辨率：米、像素/米共同确定 BEV 大小；可选长宽比检查也使用米制比例。角点仍用处理图像素。
  double field_w_m = 2.00;
  double field_h_m = 1.00;
  double bev_px_per_m = 400.0;
  int bev_width = 0;                     // >0 直接指定；有效尺寸须在 16..4096
  int bev_height = 0;                    // >0 直接指定；同上

  // ---- [分组] 颜色判据：Lab 色相扇区 + 相对色度 C/L ----
  // 色相角 atan2(b,a)（度，逆时针为正），落在 [lo,hi] 扇区内，且 C=|a|+|b| ≥ ratio×L（L 0..100）。
  // 阴影下 L 与 C 近似等比例下降，比值基本不变；扇区宽度须在 (0,180) 度。
  // 历史真机样本（43 帧，非本轮验收）：紫框 a≈18-22、b≈-20..-34（约 -45°..-62°）；
  // 红色地面 a≈53-60、b≈+35..+46（约 33°..40°）。均为待标定初值。
  double purple_hue_lo_deg = -80.0;
  double purple_hue_hi_deg = -15.0;
  double purple_chroma_ratio_min = 0.5;
  double purple_L_min = 8.0;
  double purple_L_max = 96.0;

  // ---- [分组] 形态学: 闭合连通断裂胶带 + 膨胀补断口（尺寸按处理分辨率的像素计） ----
  int morph_close_ksize = 5;
  int morph_close_iter = 2;
  int morph_dilate_ksize = 3;
  int morph_dilate_iter = 2;

  // ---- [分组] 候选过滤: 哪些紫色轮廓能进入拟合 ----
  // 面积占比相对当前搜索图（全图或跟踪 ROI），不是固定相对原始相机图。
  double min_area_frac = 0.004;
  double min_rect_ratio_lo = 1.2;
  double min_rect_ratio_hi = 8.0;        // 远距离斜视时图像里的长宽比可以到 5~7:1
  int max_border_touch = 2;
  // 凸包面积/最小外接矩形面积。紫框断裂成 C 形时"轮廓填充率"只有 0.28-0.34,
  // 会把真场地筛掉; 凸包仍然包住整个场地 (0.7-0.9), 所以用凸包, 门限 0.45。
  double min_fill_ratio = 0.45;
  int max_candidates_try = 4;

  // ---- [分组] 边拟合: 分边取点 + 鲁棒直线 ----
  // 四边拟合：每条边忽略端点、分箱取最外侧点，再拟合直线。
  double corner_exclude_frac = 0.06;      // 忽略边端点附近，避免相邻边串入拟合
  int edge_bins = 32;                    // 每条边沿边分箱数 (每箱只留最外侧点)
  double edge_band_frac = 0.18;          // 允许的"外侧"距离 (占边长比例)
  double fit_outlier_px = 2.0;          // 到拟合直线的距离，单位为处理图像素；ROI 平移不改变它
  int fit_iterations = 4;
  int min_side_points = 8;
  int min_side_inliers = 6;
  double prior_corner_tol_frac = 0.12;

  // ---- [分组] 四边形准入: 拟合结果是否算合法场地四边形 ----
  // 四边形检查：凸性/自交、面积、边长退化与上下/左右边长比，可选长宽比。
  double quad_min_area_frac = 0.004;
  // 透视下的对边长度比可以很夸张 (实测真机帧到 4.0-5.3), 所以放到 0.2..6.0;
  // 只用来挡住"明显不像四边形"的拟合, 不能当长宽比检查用。
  double quad_side_ratio_lo = 0.20;
  double quad_side_ratio_hi = 6.00;
  bool use_aspect_check = false;
  double quad_max_aspect_err = 2.0;

  // ---- [分组] 黑带与分区: 逐帧实测 split_x，跨帧保持由桥接层按时间处理 ----
  // 黑带直线：BEV 内部行中相对两侧内部颜色的局部色度下陷，投票+稳健拟合成唯一直线；
  // 不用全局灰度阈值，反光提亮的黑带仍可测；平滑上下端点，偏心只保留诊断。
  double black_search_lo = 0.14;
  double black_search_hi = 0.86;
  double black_row_lo = 0.12;
  double black_row_hi = 0.88;
  double black_min_depth = 0.35;         // 色度相对同行两侧内部颜色的下陷深度下限 1-C/C_side
  double black_conf_depth = 0.70;        // 平均下陷到该深度算满置信度
  double black_score_frac = 0.60;        // 沿线支持比例下限；三分段各须达到其一半，支持跨度须达到内部行的该比例
  double black_max_width_frac = 0.06;    // 黑带最大宽度/BEV 宽度；其一半也是两侧参照段偏移与长度
  double black_max_lift = 0.25;          // 黑带亮度最多高出两侧较亮参照的比例，更亮视为高光条带
  double black_max_offset_frac = 0.04;     // 黑带中点偏离几何中心超过该比例时标记 suspect，仍使用可靠实测

  // ---- [分组] 内部颜色证据：默认红色，blue 模式使用下方独立扇区 ----
  // 内部颜色与紫色扇区独立判定，并不显式减去紫色掩膜；扇区应避免与紫框重叠。
  double red_hue_lo_deg = -12.0;
  double red_hue_hi_deg = 46.0;
  double red_chroma_ratio_min = 0.6;
  double red_L_min = 15.0;
  // false 用上面的红色扇区。true 改用 blue_*，紫框和其余几何不变。
  // red_mask/red_inside/red_ratio 等名称沿用旧接口，蓝色模式下实际表示所选内部颜色的证据。
  bool use_blue_interior = false;
  double blue_hue_lo_deg = -140.0;
  double blue_hue_hi_deg = -80.0;
  double blue_chroma_ratio_min = 0.6;
  double blue_L_min = 12.0;
  // 内部掩膜开运算核边长（像素，奇数，1 不做）；去掉紫框与地面混色形成的一两像素细边。
  int interior_open_ksize = 3;
  double bev_border_margin_frac = 0.04;

  // ---- [分组] 状态机与提交: 命中/丢失/重拟合/跟踪窗口/角点平滑 ----
  // 改状态机阈值：命中与丢失帧数直接决定提交、保持与回退 SEARCH 的时机。
  int locate_hits_needed = 2;
  int lost_frames_to_search = 6;
  int refit_every = 10;
  double track_window_scale = 1.35;
  double corner_ema_alpha = 0.45;

  // ---- 多特征评分 / 边状态 / 预测补偿 ----
  int edge_samples = 40;               // 每条边采样点数
  int edge_probe_radius = 3;           // r 至少取1；沿边法线探测0、±r/2、±r，任一有紫色即命中
  double edge_support_min = 0.25;      // 紫色探测支持度合格门槛；实测边数另由直线拟合决定
  double edge_support_strong = 0.35;   // 仅两边探测合格时要求次强边达标；高时序一致性可豁免
  // ---- [分组] 评分权重: total = 各分项加权和, 权重之和不要求等于 1 ----
  double w_purple = 0.30;              // 评分权重: 紫框证据
  double w_temporal = 0.30;            // 评分权重: 上一帧一致性
  double w_red_inside = 0.20;          // 评分权重: 内部红色
  double w_red_outside = 0.15;         // 评分权重: 外部不是红色
  double w_geometry = 0.05;            // 评分权重: 形状合理
  double min_total_score = 0.45;       // 提交所需最低总分
  double min_purple_score = 0.28;      // 提交所需最低紫框证据 (实测好帧 0.44-0.52, 坏帧 0.02-0.06)
  double purple_lost_threshold = 0.15;    // 拒绝候选的紫证据低于此值可清历史；跳变加速另用它检查旧几何
  double red_inside_min = 0.30;        // 内部红色占比下限
  int red_cells_min = 4;               // 配置网格中至少几格满足所选内部颜色占比
  double red_outside_max = 0.25;       // 外侧红色风险上限 (避免某一条假边深入场内)
  double expand_frac = 0.08;           // 外部检测圈的扩张比例
  int red_grid_cols = 4;               // 内部红色网格 4x2
  int red_grid_rows = 2;
  double red_cell_min = 0.10;          // 单格算"有红色"的门限
  double pred_velocity_gain = 0.7;     // 预测: last + gain*(last-prev)
  double max_jump_frac = 0.35;         // 与预测的平均角点距离/对角线 超过即视为"换了地方"
  double jump_accept_frac = 0.35;     // 待确认候选之间的跳变上限，小于该值视为同一个新位置
  int switch_confirm_frames = 2;       // 大跳变候选需要连续确认几帧才切换
  double fast_switch_purple = 0.45;    // 当前跟踪几何的紫证据低于 lost 阈值时,
  double fast_switch_red_inside = 0.45;    // 若新候选足够好 (p/ri/ro) 就允许立即切换
  double fast_switch_red_outside = 0.12;
  int max_hold_frames = 5;             // 普通候选拒绝累计至 > 此值时清历史；无候选/跳变等待另走各自分支
};

// FieldConfig 字段清单的唯一副本：节点参数声明/读取与离线 JSON 序列化共用。
// 新增字段先改这里，再同步统一配置、validateFieldConfig 与参数映射；不要复制第二份名单。
#define VISION_FIELD_PARAMETERS(X) \
  X(int, proc_width) \
  X(int, proc_height) \
  X(double, field_w_m) \
  X(double, field_h_m) \
  X(double, bev_px_per_m) \
  X(int, bev_width) \
  X(int, bev_height) \
  X(double, purple_hue_lo_deg) \
  X(double, purple_hue_hi_deg) \
  X(double, purple_chroma_ratio_min) \
  X(double, purple_L_min) \
  X(double, purple_L_max) \
  X(int, morph_close_ksize) \
  X(int, morph_close_iter) \
  X(int, morph_dilate_ksize) \
  X(int, morph_dilate_iter) \
  X(double, min_area_frac) \
  X(double, min_rect_ratio_lo) \
  X(double, min_rect_ratio_hi) \
  X(int, max_border_touch) \
  X(double, min_fill_ratio) \
  X(int, max_candidates_try) \
  X(double, corner_exclude_frac) \
  X(int, edge_bins) \
  X(double, edge_band_frac) \
  X(double, fit_outlier_px) \
  X(int, fit_iterations) \
  X(int, min_side_points) \
  X(int, min_side_inliers) \
  X(double, prior_corner_tol_frac) \
  X(double, quad_min_area_frac) \
  X(double, quad_side_ratio_lo) \
  X(double, quad_side_ratio_hi) \
  X(bool, use_aspect_check) \
  X(double, quad_max_aspect_err) \
  X(double, black_search_lo) \
  X(double, black_search_hi) \
  X(double, black_row_lo) \
  X(double, black_row_hi) \
  X(double, black_min_depth) \
  X(double, black_conf_depth) \
  X(double, black_score_frac) \
  X(double, black_max_width_frac) \
  X(double, black_max_lift) \
  X(double, black_max_offset_frac) \
  X(double, red_hue_lo_deg) \
  X(double, red_hue_hi_deg) \
  X(double, red_chroma_ratio_min) \
  X(double, red_L_min) \
  X(bool, use_blue_interior) \
  X(double, blue_hue_lo_deg) \
  X(double, blue_hue_hi_deg) \
  X(double, blue_chroma_ratio_min) \
  X(double, blue_L_min) \
  X(int, interior_open_ksize) \
  X(double, bev_border_margin_frac) \
  X(int, locate_hits_needed) \
  X(int, lost_frames_to_search) \
  X(int, refit_every) \
  X(double, track_window_scale) \
  X(double, corner_ema_alpha) \
  X(int, edge_samples) \
  X(int, edge_probe_radius) \
  X(double, edge_support_min) \
  X(double, edge_support_strong) \
  X(double, w_purple) \
  X(double, w_temporal) \
  X(double, w_red_inside) \
  X(double, w_red_outside) \
  X(double, w_geometry) \
  X(double, min_total_score) \
  X(double, min_purple_score) \
  X(double, purple_lost_threshold) \
  X(double, red_inside_min) \
  X(int, red_cells_min) \
  X(double, red_outside_max) \
  X(double, expand_frac) \
  X(int, red_grid_cols) \
  X(int, red_grid_rows) \
  X(double, red_cell_min) \
  X(double, pred_velocity_gain) \
  X(double, max_jump_frac) \
  X(double, jump_accept_frac) \
  X(int, switch_confirm_frames) \
  X(double, fast_switch_purple) \
  X(double, fast_switch_red_inside) \
  X(double, fast_switch_red_outside) \
  X(int, max_hold_frames)


// FieldResult：单帧结果的完整快照 —— 几何（corners/bev/单应矩阵）、左右分区（split_x 与可信度）、质量与状态机信息、调试用的被拒候选与耗时。
// 保持期间也可能带历史 corners/bev；非空 BEV 或 state=TRACK 不代表本帧有效，使用前看 ok/committed。
struct FieldResult
{
  bool ok = false;
  std::string state = "SEARCH";        // SEARCH / LOCATE / TRACK
  std::string quality = "unknown";     // 可为 good/degraded/predicted/prior_fallback/hold/overflow/none 等诊断值
  std::string reason;
  std::vector<cv::Point2f> corners;    // TL,TR,BR,BL (处理分辨率坐标)
  cv::Mat bev;                         // 透视矫正结果
  cv::Mat homography;                  // 处理分辨率 -> 鸟瞰图
  cv::Mat homography_inv;
  double split_x = 0.0;                // 本帧实测黑带线在 BEV 中间行的 x；斜线的上下端点可不同
  double split_measured = 0.0;         // 原始黑带线中点；无测量时为 0，须同时检查 split_valid/conf
  bool split_suspect = false;          // 黑带偏离几何中线的诊断，不再据此用中线替代实测
  double split_conf = 0.0;
  bool split_valid = false;            // 本帧具有完整且唯一的黑带证据；外框 ok 不等于分区有效
  double split_top_x = 0.0, split_bottom_x = 0.0; // 本帧实测 BEV 顶/底边交点，正式几何共用这两个端点
  double red_ratio = 0.0;
  double quad_area = 0.0;
  double approx_field_h_px = 0.0;

  // ---- 边状态 / 评分 / 提交信息 ----
  EdgeState edge_state[4] = {EdgeState::LOST, EdgeState::LOST, EdgeState::LOST, EdgeState::LOST};
  int measured_edges = 0;              // 本帧真正测到的边数 (0..4)
  CandidateScore score;                // 选中候选的评分明细
  double jump_ratio = 0.0;             // 与上一帧预测的平均角点距离 / 对角线
  bool committed = false;              // 本帧角点已被接受并写入历史；LOCATE 也可为 true，保持时为 false
  bool switched = false;               // 确认计数达标后切换新位置；证据充分时可加速确认
  std::string hold_reason;             // 普通硬门槛拒绝原因；其他未提交分支可能仅填写reason
  // 普通硬门槛拒绝时的最佳候选；跳变待确认分支不填此字段
  std::vector<cv::Point2f> candidate_corners;     // 四个处理图角点；并非所有未提交分支都有值

  // 毫秒：resize为准备；detect为搜索；warp还含搜索后状态处理至透视完成；split为其后余量。
  // 分段不是互相独立的算子计时，提前拒绝帧可能保持初值0；不能用它们代替端到端延迟。
  double ms_resize = 0.0, ms_detect = 0.0, ms_warp = 0.0, ms_split = 0.0;
  double ms_total = 0.0;
};

// 一个实例保存一条相机序列的历史；按帧序调用，不提供并发保护，换会话或图像尺寸后由调用方 reset()。
class FieldLocator {
public:
  FieldLocator() = default;
  explicit FieldLocator(const FieldConfig & c)
  : cfg_(c) {}

  FieldConfig & cfg() {return cfg_;}
  const FieldConfig & cfg() const {return cfg_;}

  FieldResult update(const cv::Mat & bgr);
  void reset();

  // 供外部复用的小工具
  static cv::Mat purpleMask(const cv::Mat & bgr, const FieldConfig & cfg);
  static cv::Mat redMask(const cv::Mat & bgr, const FieldConfig & cfg);
  // 搜索用证据：segment 后对内部掩膜做 interior_open_ksize 开运算，去掉混色细边。
  static void segmentEvidence(
    const cv::Mat & bgr, const FieldConfig & cfg, cv::Mat & purple, cv::Mat & interior);
  // 一次 Lab 转换同时得到紫色和所选内部颜色掩膜；purple 与 red 须是不同的输出。
  static void segment(
    const cv::Mat & bgr, const FieldConfig & cfg, cv::Mat & purple,
    cv::Mat & red);
  static double redRatioInBev(const cv::Mat & bev, const FieldConfig & cfg);

private:
  // 搜索结果与帧内证据分开：前者保存本次候选，后者按需缓存同一帧的全图掩膜。
  struct SearchResult;
  struct FrameEvidence;
  SearchResult searchRegion(FrameEvidence &, const cv::Mat &, cv::Point2f) const;
  SearchResult findCandidate(FrameEvidence &, bool force_full) const;
  bool prepareFrame(const cv::Mat &, cv::Mat &, FieldResult &);
  void updateTracking(FrameEvidence &, SearchResult &, FieldResult &);
  // 生成输出平滑黑带上下端点；即使本帧未提交角点，也可能输出历史 BEV 诊断。
  void buildOutput(FrameEvidence &, SearchResult &, FieldResult &,
    bool ok, const std::string & quality, const std::string & why);

  FieldConfig cfg_{};
  std::string state_ = "SEARCH";
  // hits_ 决定定位确认；lost_ 统计无候选，hold_frames_ 统计有候选但未过门槛，两者独立老化。
  int hits_ = 0, lost_ = 0, frame_idx_ = 0, hold_frames_ = 0;
  bool has_corners_ = false;
  std::vector<cv::Point2f> corners_;            // 当前使用的四边形 (处理分辨率)
  std::vector<cv::Point2f> last_good_, prev_good_;     // 最近两次成功提交的角点
  std::vector<cv::Point2f> pending_;            // 大跳变候选（门槛为 max(2, switch_confirm_frames) 次确认）
  int pending_hits_ = 0;

  std::vector<cv::Point2f> predictQuad() const;     // last + gain*(last-prev)
};

// 在图上画出调试信息 (四边形/状态/左右分区), 返回可视化图。
cv::Mat drawDebug(const cv::Mat & small, const FieldResult & res);

// FieldConfig 值由统一装配注入；保留只读诊断摘要，不接受旧文本配置入口。
std::string configToString(const FieldConfig & cfg);

// 非法尺寸或算法参数抛出 std::invalid_argument，必须在开始逐帧处理前调用。
void validateFieldConfig(const FieldConfig & config);
