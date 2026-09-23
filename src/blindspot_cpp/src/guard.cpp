#include "blindspot/guard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace blindspot
{

double polygon_sigma(const Eigen::Ref<const Eigen::MatrixX2d> & s)
{
  const Eigen::Index n = s.rows();
  if (n < 3) {
    return 0.0;
  }
  // Shoelace: sum over edges of (x_i * y_{i+1} - y_i * x_{i+1}), cyclic.
  double cross = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Eigen::Index j = (i + 1) % n;
    cross += s(i, 0) * s(j, 1) - s(i, 1) * s(j, 0);
  }
  const double area = 0.5 * std::abs(cross);
  return std::sqrt(std::max(area, 0.0));
}

std::string GuardReading::str() const
{
  char buf[192];
  std::snprintf(
    buf, sizeof(buf), "signal=%.4e threshold=%.4e margin=%.2fx n=%d partition=%s",
    signal, threshold, margin, n_features, decision ? "ok" : "DROP");
  return std::string(buf);
}

Calibration::Calibration(const std::map<int, double> & thresholds, nlohmann::json metadata)
: thresholds_(thresholds), metadata_(std::move(metadata))
{
  if (thresholds_.empty()) {
    throw std::invalid_argument("a calibration needs at least one threshold");
  }
}

double Calibration::threshold_for(int n) const
{
  const auto exact = thresholds_.find(n);
  if (exact != thresholds_.end()) {
    return exact->second;
  }
  // std::map is ordered, so rbegin() is the largest key and begin() the
  // smallest. Above the table we hold the top threshold; inside it we take
  // the nearest count BELOW n, never above, because a subset's statistic is
  // bounded above by the full set's.
  if (n > thresholds_.rbegin()->first) {
    return thresholds_.rbegin()->second;
  }
  const auto at_or_after = thresholds_.lower_bound(n);
  if (at_or_after == thresholds_.begin()) {
    return thresholds_.begin()->second;
  }
  return std::prev(at_or_after)->second;
}

nlohmann::json Calibration::to_json() const
{
  nlohmann::json thr = nlohmann::json::object();
  for (const auto & [n, v] : thresholds_) {
    thr[std::to_string(n)] = v;
  }
  return nlohmann::json{
    {"schema", kCalibrationSchema},
    {"thresholds", thr},
    {"metadata", metadata_}};
}

Calibration Calibration::from_json(const nlohmann::json & j)
{
  const std::string schema = j.value("schema", std::string{});
  if (schema != kCalibrationSchema) {
    throw std::invalid_argument(
            "not a blindspot calibration file (schema='" + schema + "')");
  }
  std::map<int, double> thresholds;
  for (const auto & [k, v] : j.at("thresholds").items()) {
    thresholds[std::stoi(k)] = v.get<double>();
  }
  return Calibration(thresholds, j.value("metadata", nlohmann::json::object()));
}

FeatureGuard::FeatureGuard(Calibration calibration)
: calibration_(std::move(calibration)) {}

FeatureGuard FeatureGuard::load(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error(
            "cannot open calibration '" + path + "'.\n"
            "This node ships no default threshold. The threshold carries the scale of "
            "your target and viewing distance (measured: 3.9e-03 for one target, "
            "1.7e-02 for another), so no global constant exists.\n"
            "Calibrate on a KNOWN-GOOD target:\n"
            "    ros2 run blindspot_cpp calibrate --geometry target.json "
            "--goal-pose pose.json -o my_target.json\n"
            "Do NOT calibrate on the target you are currently servoing to unless you "
            "have verified it is healthy: calibrating in situ on a degenerate target "
            "makes the degeneracy the norm and silently disables the guard.");
  }
  nlohmann::json j;
  f >> j;
  return FeatureGuard(Calibration::from_json(j));
}

GuardReading FeatureGuard::evaluate(const Eigen::Ref<const Eigen::MatrixX2d> & s) const
{
  if (s.cols() != 2) {
    std::ostringstream os;
    os << "expected an (N, 2) array of normalised image coordinates, got ("
       << s.rows() << ", " << s.cols() << ")";
    throw std::invalid_argument(os.str());
  }
  const int n = static_cast<int>(s.rows());
  const double thr = calibration_.threshold_for(n);
  if (n < 3) {
    // fewer than three points have no polygon at all, so there is nothing for
    // the partition's area feature to stand on.
    return GuardReading{0.0, thr, 0.0, n, false};
  }
  const double sig = polygon_sigma(s);
  const double margin = thr > 0.0 ? sig / thr : std::numeric_limits<double>::infinity();
  return GuardReading{sig, thr, margin, n, sig > thr};
}

}  // namespace blindspot
