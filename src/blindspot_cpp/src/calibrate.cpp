#include "blindspot/calibrate.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace blindspot
{

namespace
{

Eigen::Matrix3d rot_z(double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  Eigen::Matrix3d R;
  R << c, -s, 0.0,
    s, c, 0.0,
    0.0, 0.0, 1.0;
  return R;
}

Eigen::Matrix3d rot_x(double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  Eigen::Matrix3d R;
  R << 1.0, 0.0, 0.0,
    0.0, c, -s,
    0.0, s, c;
  return R;
}

Eigen::Matrix3d rot_y(double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  Eigen::Matrix3d R;
  R << c, 0.0, s,
    0.0, 1.0, 0.0,
    -s, 0.0, c;
  return R;
}

}  // namespace

double percentile_linear(std::vector<double> & values, double q)
{
  if (values.empty()) {
    throw std::invalid_argument("percentile of an empty sample");
  }
  std::sort(values.begin(), values.end());
  if (values.size() == 1) {
    return values.front();
  }
  // numpy's default 'linear' method: a virtual index q/100 * (n - 1), then
  // linear interpolation between the two neighbouring order statistics.
  const double pos = (q / 100.0) * static_cast<double>(values.size() - 1);
  const double lo = std::floor(pos);
  const double hi = std::ceil(pos);
  const double frac = pos - lo;
  const double a = values[static_cast<size_t>(lo)];
  const double b = values[static_cast<size_t>(hi)];
  return a + (b - a) * frac;
}

Eigen::Matrix4d load_pose(const nlohmann::json & j)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (j.contains("cTo")) {
    const auto & rows = j.at("cTo");
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        T(r, c) = rows.at(r).at(c).get<double>();
      }
    }
    return T;
  }
  if (j.contains("R")) {
    const auto & rows = j.at("R");
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        T(r, c) = rows.at(r).at(c).get<double>();
      }
    }
  }
  const auto & t = j.at("t");
  for (int r = 0; r < 3; ++r) {
    T(r, 3) = t.at(r).get<double>();
  }
  return T;
}

Calibration calibrate(
  const Eigen::Ref<const Eigen::MatrixX3d> & points,
  const Eigen::Matrix4d & goal_pose,
  const CalibrateOptions & opts)
{
  const int n_pts = static_cast<int>(points.rows());
  if (n_pts < 3) {
    std::ostringstream os;
    os << "need at least 3 target points, got " << n_pts;
    throw std::invalid_argument(os.str());
  }
  const double Zg = goal_pose(2, 3);
  if (Zg <= 0.0) {
    std::ostringstream os;
    os << "goal pose has non-positive standoff (" << Zg << ")";
    throw std::invalid_argument(os.str());
  }
  const auto z_range = opts.z_range.value_or(std::make_pair(0.55 * Zg, 1.6 * Zg));
  const double lateral = opts.lateral.value_or(0.17 * Zg);

  // See the header: a threshold is a percentile over this sample, so it is a
  // statistical estimate, not a constant.
  std::mt19937_64 rng(opts.seed);
  std::uniform_real_distribution<double> u_ang(-M_PI, M_PI);
  std::uniform_real_distribution<double> u_tilt(-opts.tilt, opts.tilt);
  std::uniform_real_distribution<double> u_lat(-lateral, lateral);
  std::uniform_real_distribution<double> u_z(z_range.first, z_range.second);

  std::map<int, double> thresholds;
  nlohmann::json samples = nlohmann::json::object();

  for (int n_vis = 3; n_vis <= n_pts; ++n_vis) {
    std::vector<double> vals;
    vals.reserve(static_cast<size_t>(opts.n_poses));
    std::vector<int> all(static_cast<size_t>(n_pts));
    std::iota(all.begin(), all.end(), 0);

    while (static_cast<int>(vals.size()) < opts.n_poses) {
      const Eigen::Matrix3d R = rot_z(u_ang(rng)) * rot_x(u_tilt(rng)) * rot_y(u_tilt(rng));
      const Eigen::Vector3d t(u_lat(rng), u_lat(rng), u_z(rng));

      Eigen::MatrixX3d P_c(n_pts, 3);
      bool behind = false;
      for (int i = 0; i < n_pts; ++i) {
        const Eigen::Vector3d p = R * points.row(i).transpose() + t;
        if (p(2) <= 1e-6) {
          behind = true;
          break;
        }
        P_c.row(i) = p.transpose();
      }
      if (behind) {
        continue;                           // reject and redraw
      }

      Eigen::MatrixX2d s(n_pts, 2);
      s.col(0) = P_c.col(0).array() / P_c.col(2).array();
      s.col(1) = P_c.col(1).array() / P_c.col(2).array();

      if (n_vis < n_pts) {
        // A subset's statistic is bounded above by the full set's, so the
        // visible subset is drawn at random too. Sorted, to keep the polygon
        // order the same as the full set's.
        std::shuffle(all.begin(), all.end(), rng);
        std::vector<int> idx(all.begin(), all.begin() + n_vis);
        std::sort(idx.begin(), idx.end());
        Eigen::MatrixX2d sub(n_vis, 2);
        for (int i = 0; i < n_vis; ++i) {
          sub.row(i) = s.row(idx[static_cast<size_t>(i)]);
        }
        vals.push_back(polygon_sigma(sub));
      } else {
        vals.push_back(polygon_sigma(s));
      }
    }
    samples[std::to_string(n_vis)] = vals.size();
    thresholds[n_vis] = percentile_linear(vals, opts.percentile);
  }
  // Below three features there is no polygon, so the guard always fires.
  thresholds[2] = 0.0;

  nlohmann::json meta{
    {"n_points", n_pts},
    {"n_poses", opts.n_poses},
    {"percentile", opts.percentile},
    {"seed", opts.seed},
    {"z_range", nlohmann::json::array({z_range.first, z_range.second})},
    {"lateral", lateral},
    {"tilt", opts.tilt},
    {"units", "normalised image coordinates (x=X/Z, y=Y/Z, y down)"},
    {"goal_standoff_m", Zg},
    {"samples_per_count", samples},
    {"rng", "std::mt19937_64"}};

  return Calibration(thresholds, meta);
}

}  // namespace blindspot
