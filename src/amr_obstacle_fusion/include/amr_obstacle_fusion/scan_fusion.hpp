#pragma once

// 自研融合里与 ROS 无关的部分：把 base_link 下的障碍点收成一圈最小距离。
// 雷达端点和相机点都走这里，近的覆盖远的。

#include <algorithm>
#include <cmath>
#include <vector>

namespace amr_obstacle_fusion {

struct ScanBins {
  double angle_min;
  double angle_increment;
  int size;
  std::vector<float> ranges;
};

inline ScanBins makeScanBins(double angle_min, double angle_increment, int size, float empty_range) {
  ScanBins bins;
  bins.angle_min = angle_min;
  bins.angle_increment = angle_increment;
  bins.size = std::max(1, size);
  bins.ranges.assign(static_cast<size_t>(bins.size), empty_range);
  return bins;
}

inline void markMinRange(ScanBins& bins, double angle, float range) {
  if (!(range > 0.f) || !std::isfinite(range) || !std::isfinite(angle)) {
    return;
  }
  double rel = angle - bins.angle_min;
  const double two_pi = 2.0 * std::acos(-1.0);
  const double span = bins.angle_increment * static_cast<double>(bins.size);
  rel = std::fmod(rel, two_pi);
  if (rel < 0.0) {
    rel += two_pi;
  }
  // 输出是整圈时 span 约等于 2pi。+pi 与 -pi 落在同一条射线上。
  if (rel >= span) {
    if (rel - span > 1e-6) {
      return;
    }
    rel = 0.0;
  }
  int idx = static_cast<int>(rel / bins.angle_increment);
  if (idx >= bins.size) {
    idx = 0;
  }
  if (idx < 0) {
    return;
  }
  if (range < bins.ranges[static_cast<size_t>(idx)]) {
    bins.ranges[static_cast<size_t>(idx)] = range;
  }
}

inline bool insideAxisAlignedFootprint(double x, double y, double half_x, double half_y) {
  return std::fabs(x) <= half_x && std::fabs(y) <= half_y;
}

}  // namespace amr_obstacle_fusion
