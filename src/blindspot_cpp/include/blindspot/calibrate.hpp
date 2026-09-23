// Calibrate guard thresholds on a KNOWN-GOOD target.
//
//     ros2 run blindspot_cpp calibrate --geometry target.json
//         --goal-pose pose.json --poses 4000 --percentile 1 -o my_target.json
//
// --geometry  JSON: {"points": [[x, y, z], ...]} in the target frame, metres.
// --goal-pose JSON: {"cTo": [[4x4]]} or {"R": [[3x3]], "t": [x, y, z]}, the
//             transform taking target-frame points into the camera frame at
//             the desired pose. Its standoff sets the sampling envelope.
//
// The target you point this at must be HEALTHY. Calibrating on a degenerate
// one makes the degeneracy the norm: the threshold comes out below the
// degenerate target's everyday value, the guard never fires, and nothing
// looks wrong.
//
// A threshold here is a percentile over randomly drawn poses, so it is a
// statistical estimate and not a fixed constant: a different seed, or a
// different number of poses, moves it slightly. Calibrate once and ship the
// file rather than recalibrating per run when the exact value matters.
#ifndef BLINDSPOT__CALIBRATE_HPP_
#define BLINDSPOT__CALIBRATE_HPP_

#include <optional>
#include <utility>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include "blindspot/guard.hpp"

namespace blindspot
{

/// Reads {"cTo": 4x4} or {"R": 3x3, "t": [x,y,z]} into a 4x4 transform.
Eigen::Matrix4d load_pose(const nlohmann::json & j);

struct CalibrateOptions
{
  int n_poses{4000};
  double percentile{1.0};
  unsigned int seed{0};
  /// Defaults, applied when unset, are (0.55*Zg, 1.6*Zg) and 0.17*Zg.
  std::optional<std::pair<double, double>> z_range{};
  std::optional<double> lateral{};
  double tilt{0.3};
};

/// Per-feature-count thresholds from healthy poses around the goal.
///
/// Poses are drawn as
///     translation : x, y ~ U(-lateral, lateral),  z ~ U(*z_range)
///     rotation    : Rz ~ U(-pi, pi), then Rx, Ry ~ U(-tilt, tilt)
/// with defaults scaled off the goal standoff. Poses putting any feature
/// behind the camera are rejected and redrawn. For each count N the visible
/// subset is drawn at random too, because a subset's statistic is bounded
/// above by the full set's.
///
/// Throws std::invalid_argument on fewer than 3 points or a non-positive
/// goal standoff.
Calibration calibrate(
  const Eigen::Ref<const Eigen::MatrixX3d> & points,
  const Eigen::Matrix4d & goal_pose,
  const CalibrateOptions & opts = {});

/// Percentile with linear interpolation between order statistics, the same
/// convention numpy.percentile uses by default. `values` is sorted in place.
double percentile_linear(std::vector<double> & values, double q);

}  // namespace blindspot

#endif  // BLINDSPOT__CALIBRATE_HPP_
