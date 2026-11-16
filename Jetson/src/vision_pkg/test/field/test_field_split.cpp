#include "check.hpp"
#include "field_scene.hpp"
#include "harness.hpp"
#include "field/opencv/algorithm.hpp"

namespace {
cv::Mat groundBev(const field_scene::Scene & scene, const FieldConfig & config)
{
  field_scene::Truth truth;
  const auto image = field_scene::render(scene, {config.proc_width, config.proc_height}, truth);
  const int width = cvRound((field_scene::kW - 2 * field_scene::kWall) * config.bev_px_per_m);
  const int height = cvRound((field_scene::kH - 2 * field_scene::kWall) * config.bev_px_per_m);
  const std::vector<cv::Point2f> from(truth.inner.begin(), truth.inner.end());
  const std::vector<cv::Point2f> to{{0, 0}, {float(width - 1), 0},
    {float(width - 1), float(height - 1)}, {0, float(height - 1)}};
  cv::Mat result;
  cv::warpPerspective(image, result, cv::getPerspectiveTransform(from, to), {width, height});
  return result;
}
}

int main()
{
  const auto config = field_test::productionConfig();
  {
    Group group("Split accumulates the measured weak edge beside tall black blocks");
    const double heights[] = {0.03, 0.05, 0.05};
    const double positions[] = {0.18, 0.08, 0.18};
    const unsigned scenarios[] = {63, 69, 72};
    for (int test = 0; test < 3; ++test) {
      field_scene::Scene scene;
      scene.blocks = {{field_test::kBlockOffsets[1], positions[test], field_test::kBlockSize,
        heights[test], field_test::kBlack}};
      FieldLocator locator(config);
      for (int frame = 0; frame < 8; ++frame) {
        scene.seed = (7919u + scenarios[test]) * 31u + frame;
        field_scene::Truth truth;
        const auto result = locator.update(field_scene::render(scene,
          {config.proc_width, config.proc_height}, truth));
        if (frame < 4) {continue;}
        const auto bev = groundBev(scene, config);
        const auto split = field_detail::detectSplitBlack(bev, config);
        if (!result.split_valid || !split.valid) {
          std::printf("tall block case%u frame%d actual%d ideal%d\n", scenarios[test], frame,
            result.split_valid, split.valid);
        }
        CHECK(result.split_valid);
        CHECK(split.valid);
        if (split.valid) {
          CHECK(std::abs(split.top_x - 0.5 * (bev.cols - 1)) < 1.5);
          CHECK(std::abs(split.bottom_x - 0.5 * (bev.cols - 1)) < 1.5);
        }
      }
    }
  }
  {
    Group group("Split weak boundaries require a real material step");
    const int band = cvRound(config.black_width_m * 251 /
      (config.field_w_m - 2 * config.wall_thickness_m));
    for (double slope : {-0.2, 0.0, 0.2}) {
      cv::Mat bev(132, 252, CV_8UC3, field_test::kRed);
      for (int y = 0; y < bev.rows; ++y) {
        const int left = cvRound(0.5 * (bev.cols - band) + slope * (y - 65.5));
        bev.row(y).colRange(left, left + band).setTo(cv::Scalar(32, 32, 35));
        bev.row(y).colRange(left + band, left + band + 40).setTo(field_test::kBlack);
      }
      const auto split = field_detail::detectSplitBlack(bev, config);
      CHECK(split.valid);
      if (split.valid) {
        CHECK(std::abs(split.top_x - (125.5 - slope * 65.5)) < 1.5);
        CHECK(std::abs(split.bottom_x - (125.5 + slope * 65.5)) < 1.5);
      }
    }
    // 恒色、渐变、随机噪声及只有一行弱阶跃均不提供完整带宽身份。
    for (int mode = 0; mode < 4; ++mode) {
      cv::Mat bev(132, 252, CV_8UC3, field_test::kRed);
      cv::RNG random(143u);
      for (int y = 0; y < bev.rows; ++y) {
        for (int x = 116; x < 146; ++x) {
          const int light = 28 + (mode == 1 ? (x - 116) / 3 : 0);
          auto & pixel = bev.at<cv::Vec3b>(y, x);
          for (int channel = 0; channel < 3; ++channel) {
            pixel[channel] = cv::saturate_cast<uchar>(light + (channel == 2 ? 2 : 0) +
              (mode >= 2 ? random.gaussian(1.0) : 0));
          }
        }
      }
      if (mode == 3) {bev.row(65).colRange(124, 146).setTo(cv::Scalar(20, 20, 22));}
      CHECK(!field_detail::detectSplitBlack(bev, config).valid);
    }
    // 两级灰度阶梯之间的平坦像素不是材质边界。平台跨四像素求差会把它拼成
    // 标定宽度的假黑带；各真实阶梯附近的另一阶梯则是原有局部噪声判据的反证。
    for (bool rising : {false, true}) {
      cv::Mat stepped(132, 252, CV_8UC3, field_test::kRed);
      for (int x = 116; x < stepped.cols; ++x) {
        const int level = x < 123 ? 40 : x < 125 ? 34 : 28;
        const int value = rising ? 68 - level : level;
        stepped.col(x).setTo(cv::Scalar::all(value));
      }
      CHECK(!field_detail::detectSplitBlack(stepped, config).valid);
    }
  }
  {
    Group group("Split same-frame visible edge beside touching blocks");
    for (const auto & colour : {field_test::kBlack, field_test::kWhite, field_test::kBlue}) {
      for (double y : {0.18, 0.28}) {
        field_scene::Scene scene;
        scene.blocks = {{field_test::kBlockOffsets[1], y, field_test::kBlockSize, 0.01, colour}};
        const auto bev = groundBev(scene, config);
        const auto split = field_detail::detectSplitBlack(bev, config);
        if (!split.valid) {std::printf("touch block BGR %.0f/%.0f/%.0f y=%.2f rejected\n", colour[0], colour[1], colour[2], y);}
        CHECK(split.valid);
        if (split.valid) {
          if (std::abs(split.bottom_x - 0.5 * (bev.cols - 1)) >= 1.5) {
            std::printf("touch block BGR %.0f/%.0f/%.0f y=%.2f measured %.3f:%.3f\n",
              colour[0], colour[1], colour[2], y, split.top_x, split.bottom_x);
          }
          CHECK(std::abs(split.top_x - 0.5 * (bev.cols - 1)) < 1.5);
          CHECK(std::abs(split.bottom_x - 0.5 * (bev.cols - 1)) < 1.5);
        }
      }
    }
  }
  {
    Group group("Split glare preserves actual narrow divider");
    field_scene::Scene scene;
    scene.glare = 1;
    scene.glare_on_divider = true;
    scene.cam_x = -0.03;
    scene.cam_d -= 0.04;
    for (int frame : {4, 7}) {
      scene.seed = (7919u + 11u) * 31u + frame;
      const auto bev = groundBev(scene, config);
      const auto split = field_detail::detectSplitBlack(bev, config);
      CHECK(split.valid);
      if (split.valid) {
        CHECK(std::abs(split.top_x - 0.5 * (bev.cols - 1)) < 1.5);
        CHECK(std::abs(split.bottom_x - 0.5 * (bev.cols - 1)) < 1.5);
      }
    }
  }
  {
    Group group("Split ordinary glare spots preserve measured divider");
    for (int seed = 0; seed < 3; ++seed) {
      field_scene::Scene scene;
      scene.glare = 1;
      scene.cam_x += field_test::kSeedCamShiftX * (seed - 1);
      scene.cam_d += field_test::kSeedCamShiftD * (seed - 1);
      for (int frame = 4; frame < 8; ++frame) {
        scene.seed = (unsigned(seed + 1) * 7919u + 10u) * 31u + frame;
        const auto bev = groundBev(scene, config);
        const auto split = field_detail::detectSplitBlack(bev, config);
        CHECK(split.valid);
        if (split.valid) {
          CHECK(std::abs(split.top_x - 0.5 * (bev.cols - 1)) < 1.5);
          CHECK(std::abs(split.bottom_x - 0.5 * (bev.cols - 1)) < 1.5);
        }
      }
    }
  }
  {
    Group group("Split single slopes and crossing dividers retain distinct hypotheses");
    const int band_width = cvRound(config.black_width_m * 251 /
      (config.field_w_m - 2 * config.wall_thickness_m));
    for (double slope : {0.1, 0.15, 0.2, 0.25}) {
      for (double sign : {-1.0, 1.0}) {
        cv::Mat single(132, 252, CV_8UC3, field_test::kRed);
        for (int y = 0; y < single.rows; ++y) {
          const int left = cvRound(0.5 * (single.cols - band_width) +
            sign * slope * (y - 0.5 * (single.rows - 1)));
          single.row(y).colRange(left, left + band_width).setTo(field_test::kBlack);
        }
        const auto split = field_detail::detectSplitBlack(single, config);
        CHECK(split.valid);
        if (split.valid) {
          CHECK(std::abs(split.top_x - (125.5 - sign * slope * 65.5)) < 1.5);
          CHECK(std::abs(split.bottom_x - (125.5 + sign * slope * 65.5)) < 1.5);
        }
      }
      cv::Mat bev(132, 252, CV_8UC3, field_test::kRed);
      for (int y = 0; y < bev.rows; ++y) {
        for (double sign : {-1.0, 1.0}) {
          const int left = cvRound(0.5 * (bev.cols - 1 - band_width) +
            sign * slope * (y - 0.5 * (bev.rows - 1)));
          bev.row(y).colRange(left, left + band_width).setTo(field_test::kBlack);
        }
      }
      CHECK(!field_detail::detectSplitBlack(bev, config).valid);
    }
  }
  {
    Group group("Split light-gray divider and horizontal blur remain measurable");
    for (int scenario : {22, 23, 32}) {
      for (int seed = 0; seed < 3; ++seed) {
        field_scene::Scene scene;
        scene.cam_x += field_test::kSeedCamShiftX * (seed - 1);
        scene.cam_d += field_test::kSeedCamShiftD * (seed - 1);
        if (scenario == 32) {scene.motion_blur = 21;}
        else {scene.divider_bgr = field_test::kLightGrayDivider;}
        if (scenario == 23) {scene.exposure = 0.7;}
        for (int frame = 4; frame < 8; ++frame) {
          scene.seed = (unsigned(seed + 1) * 7919u + scenario) * 31u + frame;
          const auto bev = groundBev(scene, config);
          const auto split = field_detail::detectSplitBlack(bev, config);
          CHECK(split.valid);
          if (split.valid) {
            CHECK(std::abs(split.top_x - 0.5 * (bev.cols - 1)) < 1.5);
            CHECK(std::abs(split.bottom_x - 0.5 * (bev.cols - 1)) < 1.5);
          }
        }
      }
    }
  }
  {
    Group group("Split red/blue lighting and shadow");
    const int band_width = cvRound(config.black_width_m * 251 /
      (config.field_w_m - 2 * config.wall_thickness_m));
    const int band_left = (252 - band_width) / 2;
    const double band_center = band_left + 0.5 * (band_width - 1);
    for (bool blue : {false, true}) {
      auto colours = config;
      colours.use_blue_interior = blue;
      // 内部蓝色须落在生产 blue_hue 扇区；物块用的 kBlue 是另一种、更偏紫的蓝色。
      const cv::Scalar interior = blue ? cv::Scalar(170, 110, 30) : field_test::kRed;
      for (double exposure : {0.45, 1.0, 1.6}) {
        for (bool shadow : {false, true}) {
          cv::Mat bev(132, 252, CV_8UC3, interior);
          cv::rectangle(bev, {band_left, 0, band_width, bev.rows}, field_test::kBlack, cv::FILLED);
          if (shadow) {bev(cv::Rect(0, 40, bev.cols, 45)) *= 0.45;}
          bev.convertTo(bev, -1, exposure);
          const auto split = field_detail::detectSplitBlack(bev, colours);
          CHECK(split.valid);
          if (split.valid) {
            CHECK(std::abs(split.top_x - band_center) < 1.5);
            CHECK(std::abs(split.bottom_x - band_center) < 1.5);
          }
        }
      }
      cv::Mat bev(132, 252, CV_8UC3, interior);
      cv::rectangle(bev, {band_left, 0, band_width, bev.rows}, field_test::kBlack, cv::FILLED);
      for (int y = 0; y < bev.rows; ++y) {
        for (int x = 0; x < bev.cols; ++x) {
          const double dx = (x - 126.0) / 25.0, dy = (y - 70.0) / 35.0;
          const double glare = 0.7 * std::exp(-0.5 * (dx * dx + dy * dy));
          auto & pixel = bev.at<cv::Vec3b>(y, x);
          for (int channel = 0; channel < 3; ++channel) {
            pixel[channel] = cv::saturate_cast<uchar>((1 - glare) * pixel[channel] + glare * 240);
          }
        }
      }
      const auto split = field_detail::detectSplitBlack(bev, colours);
      CHECK(split.valid);
      if (split.valid) {
        CHECK(std::abs(split.top_x - band_center) < 1.5);
        CHECK(std::abs(split.bottom_x - band_center) < 1.5);
      }
      cv::Mat hidden(132, 252, CV_8UC3, interior);
      cv::rectangle(hidden, {115, 0, 22, hidden.rows}, field_test::kBlack, cv::FILLED);
      CHECK(!field_detail::detectSplitBlack(hidden, colours).valid);
      cv::Mat ambiguous(132, 252, CV_8UC3, interior);
      cv::rectangle(ambiguous, {90, 0, band_width, ambiguous.rows}, field_test::kBlack, cv::FILLED);
      cv::rectangle(ambiguous, {150, 0, band_width, ambiguous.rows}, field_test::kBlack, cv::FILLED);
      CHECK(!field_detail::detectSplitBlack(ambiguous, colours).valid);
    }
  }
  {
    Group group("Split calibrated width uses actual BEV dimensions");
    for (int width : {252, 504}) {
      auto sized = config;
      sized.bev_width = width;
      sized.bev_height = 132;
      const int band = cvRound(sized.black_width_m * (width - 1) /
        (sized.field_w_m - 2 * sized.wall_thickness_m));
      const int left = (width - band) / 2;
      cv::Mat bev(sized.bev_height, width, CV_8UC3, field_test::kRed);
      cv::rectangle(bev, {left, 0, band, bev.rows}, field_test::kBlack, cv::FILLED);
      // Three actual two-edge rows identify the band; below them the right edge touches a black block.
      const int block_top = int(sized.black_row_lo * bev.rows) + 3;
      cv::rectangle(bev, {left + band, block_top, 3 * band, bev.rows - block_top},
        field_test::kBlack, cv::FILLED);
      const auto split = field_detail::detectSplitBlack(bev, sized);
      CHECK(split.valid);
      if (split.valid) {
        const double center = left + 0.5 * (band - 1);
        CHECK(std::abs(split.top_x - center) < 1.5);
        CHECK(std::abs(split.bottom_x - center) < 1.5);
      }
      // 稀疏的双边片段宽过标定带时，不能用宽块外沿补成整条中线。
      // 多出的 3 px 仍通过旧 1.5 倍像素宽度粗筛，却超出实测边距的既有 2 px 容差。
      const int broad_band = band + 3;
      cv::Mat broad(sized.bev_height, width, CV_8UC3, field_test::kRed);
      cv::rectangle(broad, {left, 0, broad_band, broad.rows}, field_test::kBlack, cv::FILLED);
      cv::rectangle(broad, {left + broad_band, block_top, 3 * broad_band, broad.rows - block_top},
        field_test::kBlack, cv::FILLED);
      CHECK(!field_detail::detectSplitBlack(broad, sized).valid);
    }
  }
  {
    Group group("Split blue block seams cannot impersonate the calibrated divider");
    int scenario = 125;
    for (double height : {0.02, 0.03, 0.05}) {
      field_scene::Scene scene;
      scene.blocks = {{0.0, 0.18, field_test::kBlockSize, height, field_test::kBlue}};
      FieldLocator locator(config);
      for (int frame = 0; frame < 8; ++frame) {
        scene.seed = (7919u + scenario) * 31u + frame;
        field_scene::Truth truth;
        const auto result = locator.update(field_scene::render(scene,
          {config.proc_width, config.proc_height}, truth));
        if (frame < 4) {continue;}
        CHECK(!field_detail::detectSplitBlack(groundBev(scene, config), config).valid);
        CHECK(!result.split_valid);
      }
      scenario += 9;
    }
  }
  {
    Group group("Split weak edges cannot turn shaded red ground into a black divider");
    for (int seed = 0; seed < 3; ++seed) {
      field_scene::Scene scene;
      scene.tint = {0.75, 0.95, 1.25};
      scene.glare = 1; scene.shadows = 1; scene.floor = field_scene::Floor::WOOD;
      scene.blocks = {{0.0, 0.18, 0.05, 0.05, field_test::kBlack}};
      scene.cam_x += 0.03 * (seed - 1); scene.cam_d += 0.04 * (seed - 1);
      scene.seed = unsigned(seed + 1) * 7919u + 84u + seed;
      FieldLocator locator(config);
      for (int frame = 0; frame < 8; ++frame) {
        // 保持旧host场景的递推种子，覆盖其实际误报帧而非仅相似的矩阵种子。
        scene.seed = scene.seed * 31u + frame;
        field_scene::Truth truth;
        const auto result = locator.update(field_scene::render(scene,
          {config.proc_width, config.proc_height}, truth));
        if (frame < 4) {continue;}
        CHECK(!result.split_valid);
        CHECK(!field_detail::detectSplitBlack(groundBev(scene, config), config).valid);
      }
    }
  }
  {
    Group group("Split fully occluded warm block is not a divider");
    field_scene::Scene scene;
    scene.tint = field_test::kTintWarm;
    scene.awb_compensation = field_test::kAwbResidual;
    scene.glare = 1;
    scene.shadows = 1;
    scene.floor = field_scene::Floor::WOOD;
    scene.blocks = {{0.0, 0.18, 0.05, 0.05, field_test::kBlack}};
    for (int seed = 0; seed < 3; ++seed) {
      scene.cam_x = field_test::kSeedCamShiftX * (seed - 1);
      scene.cam_d = 0.75 + field_test::kSeedCamShiftD * (seed - 1);
      FieldLocator locator(config);
      for (int frame = 0; frame < 8; ++frame) {
        scene.seed = (unsigned(seed + 1) * 7919u + 37u) * 31u + frame;
        field_scene::Truth truth;
        const auto result = locator.update(field_scene::render(scene,
          {config.proc_width, config.proc_height}, truth));
        if (frame < 4) {continue;}
        const auto ground = field_detail::detectSplitBlack(groundBev(scene, config), config);
        if (ground.valid) {std::printf("occluded seed%d frame%d ground split %.2f:%.2f\n", seed, frame, ground.top_x, ground.bottom_x);}
        CHECK(!ground.valid);
        CHECK(!result.split_valid);
      }
    }
  }
  return finishTests("Field split evidence");
}
