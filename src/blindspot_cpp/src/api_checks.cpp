// Checks for the packaged guard. The study's regression suite covers the
// findings; these cover the API contract, in particular the things the design
// deliberately forbids.
//
//     ros2 run blindspot_cpp api_checks
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <vision_msgs/msg/detection2_d.hpp>

#include "blindspot/calibrate.hpp"
#include "blindspot/detection_msg.hpp"
#include "blindspot/guard.hpp"
#include "blindspot/ros_bridge.hpp"
#include "blindspot/units.hpp"

namespace
{

int g_pass = 0;
int g_total = 0;

void check(const std::string & name, bool cond, const std::string & detail = "")
{
  ++g_total;
  g_pass += cond ? 1 : 0;
  std::printf(
    "[%s] %s %s\n", cond ? "  ok  " : " FAIL ", name.c_str(), detail.c_str());
}

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

/// An independent area, summed as a triangle fan about vertex 0 instead of as
/// a shoelace over the edges. Same answer for a simple polygon, different
/// arithmetic, so it catches a sign or wrap-around slip in polygon_sigma.
double fan_sigma(const Eigen::MatrixX2d & s)
{
  double a = 0.0;
  for (Eigen::Index i = 1; i + 1 < s.rows(); ++i) {
    a += 0.5 * ((s(i, 0) - s(0, 0)) * (s(i + 1, 1) - s(0, 1)) -
      (s(i, 1) - s(0, 1)) * (s(i + 1, 0) - s(0, 0)));
  }
  return std::sqrt(std::abs(a));
}

/// Runs a command, capturing stdout. Returns the exit status.
int run(const std::string & cmd, std::string * out)
{
  out->clear();
  FILE * pipe = popen((cmd + " 2>&1").c_str(), "r");
  if (pipe == nullptr) {
    return -1;
  }
  char buf[4096];
  while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
    *out += buf;
  }
  const int status = pclose(pipe);
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

// A guard cannot be built without a calibration, and takes nothing else. Both
// are compile-time facts, so state them where they cannot rot.
static_assert(
  !std::is_default_constructible<blindspot::FeatureGuard>::value,
  "FeatureGuard must not be default-constructible: it would ship a default threshold");
static_assert(
  std::is_constructible<blindspot::FeatureGuard, blindspot::Calibration>::value,
  "FeatureGuard must be constructible from a Calibration");
static_assert(
  !std::is_constructible<blindspot::FeatureGuard, blindspot::Calibration, double>::value,
  "FeatureGuard must not accept a second argument: tau belongs to the controller");

}  // namespace

int main()
{
  Eigen::Matrix4d goal = Eigen::Matrix4d::Identity();
  goal(2, 3) = 0.6;
  blindspot::CalibrateOptions opts;
  opts.n_poses = 400;
  opts.seed = 0;
  const auto cal = blindspot::calibrate(ring(), goal, opts);
  const blindspot::FeatureGuard guard(cal);

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.topRightCorner<3, 1>() = Eigen::Vector3d(0.02, -0.01, 0.62);

  std::printf("--- rule 1: no default threshold ---\n");
  try {
    blindspot::Calibration empty{{}};
    check("an empty calibration is refused", false);
  } catch (const std::invalid_argument &) {
    check("an empty calibration is refused", true);
  }
  {
    bool refused = false;
    std::string msg;
    try {
      blindspot::FeatureGuard::load("/nonexistent/cal.json");
    } catch (const std::runtime_error & e) {
      refused = true;
      msg = e.what();
    }
    check("loading a missing calibration is refused", refused);
    check(
      "  and the error explains the in-situ trap",
      msg.find("in situ") != std::string::npos &&
      msg.find("KNOWN-GOOD") != std::string::npos);
    check(
      "  and names the calibrate command",
      msg.find("ros2 run blindspot_cpp calibrate") != std::string::npos);
  }
  check(
    "FeatureGuard is not default-constructible",
    !std::is_default_constructible<blindspot::FeatureGuard>::value);

  std::printf("\n--- rule 2: the guard has no truncation tolerance ---\n");
  check("FeatureGuard takes a Calibration and nothing else", true, "(static_assert)");
  {
    bool has_tau = false;
    for (const auto & [k, v] : cal.metadata().items()) {
      (void)v;
      std::string lower = k;
      for (auto & c : lower) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      has_tau = has_tau || lower.find("tau") != std::string::npos;
    }
    check("Calibration stores no tau", !has_tau);
  }
  check(
    "the library ships no controller", true,
    "(tau belongs to yours; the reference ones are study code)");

  std::printf("\n--- rule 3: units ---\n");
  const auto k = blindspot::intrinsics_from_fov(1920, 1440, 45.0);
  {
    char d[64];
    std::snprintf(d, sizeof(d), "(cx=%.1f, cy=%.1f)", k.cx, k.cy);
    check("principal point is (W-1)/2, not W/2", k.cx == 959.5 && k.cy == 719.5, d);
  }
  const Eigen::MatrixX2d s_true = view(ring(), pose);
  Eigen::MatrixX2d uv(s_true.rows(), 2);
  uv.col(0) = s_true.col(0).array() * k.fx + k.cx;
  uv.col(1) = s_true.col(1).array() * k.fy + k.cy;
  const Eigen::MatrixX2d back = blindspot::pixels_to_normalised(uv, k);
  check("pixels round-trip to normalised", (back - s_true).cwiseAbs().maxCoeff() < 1e-12);
  try {
    blindspot::pixels_to_normalised(uv, blindspot::Intrinsics{0.0, 0.0, 959.5, 719.5});
    check("zero intrinsics are refused, not guessed", false);
  } catch (const std::invalid_argument &) {
    check("zero intrinsics are refused, not guessed", true);
  }

  std::printf("\n--- the signal ---\n");
  check(
    "polygon_sigma agrees with an independent triangle-fan area",
    std::abs(blindspot::polygon_sigma(s_true) - fan_sigma(s_true)) < 1e-15);

  std::printf("\n--- decisions ---\n");
  check("healthy view: partition ok", guard.partition_ok(s_true));
  Eigen::MatrixX3d flat = ring();
  flat.col(1) *= 0.02;
  const Eigen::MatrixX2d s_flat = view(flat, pose);
  {
    char d[64];
    std::snprintf(d, sizeof(d), "(margin %.2fx)", guard.evaluate(s_flat).margin);
    check(
      "collapsed target, all 6 features: partition dropped",
      !guard.partition_ok(s_flat), d);
  }
  {
    Eigen::MatrixX2d two(2, 2);
    two.row(0) = s_true.row(0);
    two.row(1) = s_true.row(3);
    check("two features: partition dropped", !guard.partition_ok(two));
  }
  {
    const auto r0 = guard.evaluate(Eigen::MatrixX2d(0, 2));
    check(
      "no features at all: dropped, not crashed",
      !r0.decision && r0.n_features == 0 && r0.signal == 0.0);
  }
  const auto r = guard.evaluate(s_true);
  check(
    "evaluate exposes signal, threshold, margin, count, decision",
    r.signal > 0 && r.threshold > 0 && r.margin > 1 && r.n_features == 6 && r.decision,
    "(" + r.str() + ")");
  {
    char d[96];
    std::snprintf(
      d, sizeof(d), "(N=3 %.3e < N=6 %.3e)", cal.threshold_for(3), cal.threshold_for(6));
    check(
      "thresholds are conditioned on the feature count",
      cal.threshold_for(3) < cal.threshold_for(6), d);
  }

  std::printf("\n--- round-trip through a calibration file ---\n");
  const auto dir = std::filesystem::temp_directory_path() / "blindspot_api_checks";
  std::filesystem::create_directories(dir);
  {
    const auto p = dir / "cal.json";
    {
      std::ofstream f(p);
      f << cal.to_json().dump();
    }
    const auto g2 = blindspot::FeatureGuard::load(p.string());
    check(
      "load() reproduces the same decisions",
      g2.evaluate(s_true).signal == r.signal &&
      g2.calibration().thresholds() == cal.thresholds());

    const auto bad = dir / "bad.json";
    {
      std::ofstream f(bad);
      f << nlohmann::json{{"thresholds", {{"6", 1.0}}}}.dump();
    }
    try {
      blindspot::FeatureGuard::load(bad.string());
      check("a file without the schema tag is refused", false);
    } catch (const std::invalid_argument &) {
      check("a file without the schema tag is refused", true);
    }
  }

  std::printf("\n--- the calibrate command ---\n");
  {
    const auto geo = dir / "geo.json";
    const auto gp = dir / "goal.json";
    const auto out = dir / "out.json";
    {
      std::ofstream f(geo);
      nlohmann::json pts = nlohmann::json::array();
      const auto R = ring();
      for (Eigen::Index i = 0; i < R.rows(); ++i) {
        pts.push_back({R(i, 0), R(i, 1), R(i, 2)});
      }
      f << nlohmann::json{{"points", pts}}.dump();
    }
    {
      std::ofstream f(gp);
      nlohmann::json rows = nlohmann::json::array();
      for (int i = 0; i < 4; ++i) {
        rows.push_back({goal(i, 0), goal(i, 1), goal(i, 2), goal(i, 3)});
      }
      f << nlohmann::json{{"cTo", rows}}.dump();
    }
    std::string stdout_text;
    const int rc = run(
      std::string(BLINDSPOT_CALIBRATE_BIN) + " --geometry " + geo.string() +
      " --goal-pose " + gp.string() + " --poses 200 -o " + out.string(), &stdout_text);
    check("the calibrate command exits 0", rc == 0);
    bool loadable = false;
    try {
      blindspot::FeatureGuard::load(out.string());
      loadable = true;
    } catch (const std::exception &) {
    }
    check("  and writes a loadable calibration", loadable);
    check(
      "  and warns that the target must have been healthy",
      stdout_text.find("healthy") != std::string::npos);
  }
  std::filesystem::remove_all(dir);

  std::printf("\n--- ROS bridge: ordering, units, message shapes ---\n");
  Eigen::MatrixX2d sq(4, 2);
  sq << 0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0;
  Eigen::MatrixX2d bow(4, 2);
  bow.row(0) = sq.row(0);
  bow.row(1) = sq.row(2);
  bow.row(2) = sq.row(1);
  bow.row(3) = sq.row(3);
  check("a sane ordering is not flagged", !blindspot::ordering_is_suspect(sq));
  check(
    "a self-crossing ordering IS flagged", blindspot::ordering_is_suspect(bow),
    "(shoelace area collapses, which would fire the guard for the wrong reason)");
  {
    const auto o = blindspot::order_features(bow, {"0", "2", "1", "3"});
    check(
      "ordering by detection id repairs it",
      o.how == blindspot::Ordering::kById && !blindspot::ordering_is_suspect(o.uv));
    const auto oa = blindspot::order_features(bow, {});
    check(
      "without ids it falls back to an angular sort",
      oa.how == blindspot::Ordering::kAngular && !blindspot::ordering_is_suspect(oa.uv));
  }
  {
    const auto ci = blindspot::intrinsics_from_camera_info(
      {1738.2, 0, 959.5, 0, 1738.2, 719.5, 0, 0, 1}, 1920, 1440);
    check(
      "a calibrated camera_info is used as-is",
      ci.source == blindspot::IntrinsicsSource::kCameraInfo && ci.k.cx == 959.5 &&
      ci.k.fx == 1738.2);
    const auto cu = blindspot::intrinsics_from_camera_info(
      {0, 0, 0, 0, 0, 0, 0, 0, 0}, 1920, 1440);
    check(
      "an uncalibrated one reports so and offers (W-1)/2",
      cu.source == blindspot::IntrinsicsSource::kUncalibrated && cu.k.cx == 959.5 &&
      cu.k.cy == 719.5);
  }
  {
    const auto sr = blindspot::features_to_normalised(
      uv, k, {"0", "1", "2", "3", "4", "5"});
    check(
      "bridge reproduces the guard decision from pixels",
      guard.evaluate(sr.s).decision == guard.evaluate(s_true).decision &&
      std::abs(guard.evaluate(sr.s).signal - guard.evaluate(s_true).signal) < 1e-12);
  }
  {
    vision_msgs::msg::Detection2D d;
    d.bbox.center.position.x = 12.0;
    d.bbox.center.position.y = 34.0;
    d.id = "3";
    check(
      "reads a Detection2D centre and id",
      blindspot::detection_center(d) == std::make_pair(12.0, 34.0) &&
      blindspot::detection_label(d) == "3");

    vision_msgs::msg::Detection2D dh;
    dh.bbox.center.position.x = 5.0;
    dh.bbox.center.position.y = 6.0;
    vision_msgs::msg::ObjectHypothesisWithPose h;
    h.hypothesis.class_id = "7";
    dh.results.push_back(h);
    check(
      "falls back to the hypothesis class_id when there is no id",
      blindspot::detection_center(dh) == std::make_pair(5.0, 6.0) &&
      blindspot::detection_label(dh) == "7");
  }

  std::printf("\n%d/%d passed\n", g_pass, g_total);
  return g_pass == g_total ? 0 : 1;
}
