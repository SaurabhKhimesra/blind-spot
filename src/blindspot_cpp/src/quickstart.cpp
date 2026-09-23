// Calibrates the shipped ring target and shows the guard firing on a
// degenerate view and staying quiet on a healthy one.
//
//     ros2 run blindspot_cpp quickstart
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include <Eigen/Core>

#include "blindspot/calibrate.hpp"
#include "blindspot/guard.hpp"
#include "blindspot/units.hpp"

namespace
{

Eigen::MatrixX3d ring(double r = 0.05, int n = 6)
{
  Eigen::MatrixX3d p(n, 3);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    p(i, 0) = r * std::cos(a);
    p(i, 1) = r * std::sin(a);
    p(i, 2) = 0.0;
  }
  return p;
}

Eigen::MatrixX2d view(const Eigen::MatrixX3d & points, const Eigen::Matrix4d & cTo)
{
  Eigen::MatrixX2d s(points.rows(), 2);
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const Eigen::Vector3d p =
      cTo.topLeftCorner<3, 3>() * points.row(i).transpose() + cTo.topRightCorner<3, 1>();
    s(i, 0) = p(0) / p(2);
    s(i, 1) = p(1) / p(2);
  }
  return s;
}

}  // namespace

int main()
{
  Eigen::Matrix4d goal = Eigen::Matrix4d::Identity();
  goal(2, 3) = 0.6;                       // 0.6 m straight on

  std::printf("Calibrating on the KNOWN-GOOD ring target (600 poses for speed)...\n");
  blindspot::CalibrateOptions opts;
  opts.n_poses = 600;
  opts.percentile = 1.0;
  opts.seed = 0;
  const auto cal = blindspot::calibrate(ring(), goal, opts);
  const blindspot::FeatureGuard guard(cal);

  std::printf("thresholds by visible count: {");
  bool first = true;
  for (const auto & [n, v] : cal.thresholds()) {
    std::printf("%s%d: %.3e", first ? "" : ", ", n, v);
    first = false;
  }
  std::printf("}\n\n");

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.topRightCorner<3, 1>() = Eigen::Vector3d(0.02, -0.01, 0.62);

  const Eigen::MatrixX2d healthy = view(ring(), pose);
  std::printf("1. healthy view, all 6 features\n");
  std::printf("    %s\n", guard.evaluate(healthy).str().c_str());

  Eigen::MatrixX2d two(2, 2);
  two.row(0) = healthy.row(0);
  two.row(1) = healthy.row(3);
  std::printf("2. occluded, only 2 features survive\n");
  std::printf("    %s\n", guard.evaluate(two).str().c_str());

  Eigen::MatrixX3d flat = ring();
  flat.col(1) *= 0.02;
  std::printf("3. target geometry collapsed toward a line, still all 6 features\n");
  std::printf("    %s\n", guard.evaluate(view(flat, pose)).str().c_str());

  std::printf("\n4. same thing from pixel coordinates (1920x1440, 45 deg vertical FOV)\n");
  const auto k = blindspot::intrinsics_from_fov(1920, 1440, 45.0);
  Eigen::MatrixX2d uv(healthy.rows(), 2);
  uv.col(0) = healthy.col(0).array() * k.fx + k.cx;
  uv.col(1) = healthy.col(1).array() * k.fy + k.cy;
  std::printf(
    "   principal point is (W-1)/2 = %.1f, not W/2 = %.1f\n", k.cx, 1920 / 2.0);
  std::printf(
    "    %s\n", guard.evaluate(blindspot::pixels_to_normalised(uv, k)).str().c_str());

  std::printf("\n5. the API refuses to run without a calibration\n");
  try {
    blindspot::FeatureGuard::load("/nonexistent/cal.json");
  } catch (const std::runtime_error & e) {
    const std::string msg = e.what();
    std::printf("   runtime_error: %s\n", msg.substr(0, msg.find('\n')).c_str());
  }

  std::printf("\nUse it in your own loop:\n");
  std::printf("    if (guard.partition_ok(s_visible)) v = my_partitioned_control(...);\n");
  std::printf("    else                               v = my_plain_control(...);\n");
  return 0;
}
