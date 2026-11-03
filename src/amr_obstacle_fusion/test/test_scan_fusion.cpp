#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "amr_obstacle_fusion/scan_fusion.hpp"

namespace {

int failures = 0;

void expect(bool cond, const char* text) {
  if (!cond) {
    std::cerr << "FAIL " << text << "\n";
    ++failures;
  }
}

}  // namespace

int main() {
  using amr_obstacle_fusion::insideAxisAlignedFootprint;
  using amr_obstacle_fusion::makeScanBins;
  using amr_obstacle_fusion::markMinRange;

  const double pi = std::acos(-1.0);
  const double inc = (2.0 * pi) / 8.0;
  auto bins = makeScanBins(-pi, inc, 8, std::numeric_limits<float>::infinity());
  expect(bins.size == 8, "bin count");

  markMinRange(bins, 0.0, 3.0f);
  expect(bins.ranges[4] == 3.0f, "angle 0 lands on bin 4");

  markMinRange(bins, 0.0, 1.5f);
  expect(bins.ranges[4] == 1.5f, "closer range replaces");
  markMinRange(bins, 0.0, 4.0f);
  expect(bins.ranges[4] == 1.5f, "farther range does not replace");

  markMinRange(bins, pi, 2.0f);
  markMinRange(bins, -pi, 1.0f);
  expect(bins.ranges[0] == 1.0f, "+pi and -pi share a ray");

  expect(insideAxisAlignedFootprint(0.1, -0.1, 0.42, 0.27), "inside body");
  expect(!insideAxisAlignedFootprint(0.50, 0.0, 0.42, 0.27), "in front of body");
  expect(!insideAxisAlignedFootprint(0.0, 0.30, 0.42, 0.27), "beside body");

  if (failures != 0) {
    std::cerr << failures << " failure(s)\n";
    return EXIT_FAILURE;
  }
  std::cout << "scan_fusion ok\n";
  return EXIT_SUCCESS;
}
