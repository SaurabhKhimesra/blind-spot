// The guard: a per-step answer to "is the partition safe right now".
//
// Nothing in this header knows about a controller. It takes the visible
// feature positions and returns a decision plus the numbers behind it.
//
// Two things are deliberately made impossible here.
//
//   * There is NO default threshold. A FeatureGuard cannot be constructed
//     without a Calibration produced from a KNOWN-GOOD target. Calibrating on
//     whatever the camera happens to be looking at makes the degeneracy the
//     norm and silently disables the guard - measured, the in-situ threshold
//     came out 7x lower, never fired, and the run floored while still looking
//     like it worked. There is no default constructor for that reason.
//
//   * This header has no truncation tolerance. The guard threshold and the
//     pseudo-inverse tolerance tau are different quantities answering
//     different questions, and sharing one value between them was a real bug
//     that left a 1.36x margin before the retreat fix broke. tau belongs to
//     whatever controller you run, not to the guard; see arm_law.hpp.
#ifndef BLINDSPOT__GUARD_HPP_
#define BLINDSPOT__GUARD_HPP_

#include <map>
#include <string>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

namespace blindspot
{

/// Schema tag written into, and required by, every calibration file.
inline constexpr const char * kCalibrationSchema = "blindspot/guard-calibration/1";

/// sqrt of the polygon area of the feature set, in normalised units.
///
/// An aggregate over every visible point, so independent per-point detector
/// noise largely cancels inside it: measured, 2 px of per-point noise moves
/// this by well under 1% of a typical threshold.
///
/// The shoelace sum depends on the ORDER of the points. Order them before
/// calling this - see ros_bridge.hpp.
double polygon_sigma(const Eigen::Ref<const Eigen::MatrixX2d> & s);

/// Everything behind one decision. Log this, not just the bool.
struct GuardReading
{
  double signal{0.0};
  double threshold{0.0};
  double margin{0.0};     ///< signal / threshold; < 1 means the guard fires
  int n_features{0};
  bool decision{false};   ///< true = the partition is safe to use

  std::string str() const;
};

/// Per-feature-count thresholds measured on a known-good target.
class Calibration
{
public:
  /// Throws std::invalid_argument on an empty table: a calibration with no
  /// threshold is not a calibration.
  explicit Calibration(
    const std::map<int, double> & thresholds,
    nlohmann::json metadata = nlohmann::json::object());

  /// Threshold for n visible features.
  ///
  /// Conditioned on the count because singular-value interlacing means a
  /// subset's statistic is bounded above by the full set's: calibrating at
  /// N=6 and operating at N=3 left a 1.19x margin where conditioning gives
  /// 45.9x.
  double threshold_for(int n) const;

  const std::map<int, double> & thresholds() const {return thresholds_;}
  const nlohmann::json & metadata() const {return metadata_;}

  nlohmann::json to_json() const;

  /// Throws std::invalid_argument if the schema tag is absent or wrong.
  static Calibration from_json(const nlohmann::json & j);

private:
  std::map<int, double> thresholds_;
  nlohmann::json metadata_;
};

/// Decides, per step, whether the partitioned control law is safe.
///
///     auto guard = blindspot::FeatureGuard::load("my_target.json");
///     if (guard.partition_ok(s_visible)) {
///       v = my_partitioned_control(...);
///     } else {
///       v = my_plain_control(...);
///     }
///
/// `s_visible` is an (N, 2) matrix of the CURRENTLY VISIBLE features in
/// normalised image coordinates - see units.hpp.
class FeatureGuard
{
public:
  explicit FeatureGuard(Calibration calibration);

  FeatureGuard() = delete;

  /// Reads a calibration file written by `ros2 run blindspot_cpp calibrate`.
  /// Throws std::runtime_error if it cannot be read.
  static FeatureGuard load(const std::string & path);

  /// Full reading: signal, threshold, margin, count, decision.
  /// Throws std::invalid_argument if `s_visible` is not (N, 2).
  GuardReading evaluate(const Eigen::Ref<const Eigen::MatrixX2d> & s_visible) const;

  /// True if the partitioned control law is safe this step.
  bool partition_ok(const Eigen::Ref<const Eigen::MatrixX2d> & s_visible) const
  {
    return evaluate(s_visible).decision;
  }

  const Calibration & calibration() const {return calibration_;}

private:
  Calibration calibration_;
};

}  // namespace blindspot

#endif  // BLINDSPOT__GUARD_HPP_
