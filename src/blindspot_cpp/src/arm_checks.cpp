// The folding part: the arm act, run offline against the arm's own law.
//
//     ros2 run blindspot_cpp arm_checks
//
// This includes arm_law.hpp, the same header the Gazebo nodes link, so the
// act here cannot drift from the one the arms run.
//
// A part whose markers fold toward a line in 3D is the failure the 2001
// partition cannot survive: it reads the shrinking polygon as distance and
// drives the camera into the part. sigma_6 never fires, because folding AWAY
// from the camera adds depth variation and the interaction matrix stays well
// conditioned. The area guard fires only when the fold outpaces the
// partition's own depth loop, because the lunge restores the very area the
// guard watches.
//
// The first check compares a calibrated threshold against 2.009e-01, the
// value the Gazebo run logged, with a 2% tolerance. That threshold is a
// percentile over randomly drawn poses, so it is a statistical estimate and
// not a constant: do not tighten the tolerance without re-deriving it.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "blindspot/arm_law.hpp"
#include "blindspot/calibrate.hpp"
#include "blindspot/guard.hpp"

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

// ---------------------------------------------------------------------------
// study primitives this act needs, kept local so the checks do not pull the
// whole reference controller set into the C++ package
// ---------------------------------------------------------------------------

Eigen::Matrix3d skew(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d K;
  K << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0;
  return K;
}

Eigen::Matrix4d se3_exp(const Eigen::Matrix<double, 6, 1> & xi)
{
  const Eigen::Vector3d v = xi.head<3>();
  const Eigen::Vector3d w = xi.tail<3>();
  const double th = w.norm();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (th < 1e-12) {
    T.topRightCorner<3, 1>() = v;
    return T;
  }
  const Eigen::Matrix3d W = skew(w / th);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  T.topLeftCorner<3, 3>() = I + std::sin(th) * W + (1.0 - std::cos(th)) * (W * W);
  const Eigen::Matrix3d V =
    I + ((1.0 - std::cos(th)) / th) * W + ((th - std::sin(th)) / th) * (W * W);
  T.topRightCorner<3, 1>() = V * v;
  return T;
}

Eigen::Matrix3d rot_z(double a)
{
  Eigen::Matrix3d R;
  R << std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1;
  return R;
}

Eigen::Matrix3d rot_x(double a)
{
  Eigen::Matrix3d R;
  R << 1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a);
  return R;
}

Eigen::Matrix3d rot_y(double a)
{
  Eigen::Matrix3d R;
  R << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
  return R;
}

Eigen::MatrixX3d transform_points(const Eigen::Matrix4d & T, const Eigen::MatrixX3d & P)
{
  Eigen::MatrixX3d out(P.rows(), 3);
  for (Eigen::Index i = 0; i < P.rows(); ++i) {
    out.row(i) =
      (T.topLeftCorner<3, 3>() * P.row(i).transpose() + T.topRightCorner<3, 1>())
      .transpose();
  }
  return out;
}

Eigen::MatrixX2d project(const Eigen::MatrixX3d & P_c)
{
  Eigen::MatrixX2d s(P_c.rows(), 2);
  s.col(0) = P_c.col(0).array() / P_c.col(2).array();
  s.col(1) = P_c.col(1).array() / P_c.col(2).array();
  return s;
}

Eigen::MatrixXd interaction_matrix(
  const Eigen::MatrixX2d & s, const Eigen::VectorXd & Z)
{
  Eigen::MatrixXd L(2 * s.rows(), 6);
  for (Eigen::Index i = 0; i < s.rows(); ++i) {
    const double x = s(i, 0);
    const double y = s(i, 1);
    const double Zi = Z(i);
    L.row(2 * i) << -1.0 / Zi, 0.0, x / Zi, x * y, -(1.0 + x * x), y;
    L.row(2 * i + 1) << 0.0, -1.0 / Zi, y / Zi, 1.0 + y * y, -x * y, -x;
  }
  return L;
}

/// The 6th singular value, with the spectrum padded to length 6.
///
/// Padding means that when the camera has fewer independent image constraints
/// than degrees of freedom, sigma_6 is EXACTLY zero, which is the correct
/// reading: those DOF are unobservable rather than merely ill-conditioned.
/// The naive S.tail(1) returns a healthy-looking number instead.
double sigma_6(const Eigen::MatrixXd & L)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(L);
  const Eigen::VectorXd & S = svd.singularValues();
  if (S.size() < 6) {
    return 0.0;
  }
  return S.minCoeff();
}

/// Healthy pose family shared by both calibrators, so the two rules differ
/// only in WHICH quantity they test, never in how the threshold was obtained.
std::vector<Eigen::MatrixX3d> healthy_poses(
  const Eigen::MatrixX3d & P_o, int n, unsigned int seed,
  std::pair<double, double> z_range, double lateral, double tilt)
{
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u_ang(-M_PI, M_PI);
  std::uniform_real_distribution<double> u_tilt(-tilt, tilt);
  std::uniform_real_distribution<double> u_lat(-lateral, lateral);
  std::uniform_real_distribution<double> u_z(z_range.first, z_range.second);

  std::vector<Eigen::MatrixX3d> out;
  out.reserve(static_cast<size_t>(n));
  while (static_cast<int>(out.size()) < n) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = rot_z(u_ang(rng)) * rot_x(u_tilt(rng)) * rot_y(u_tilt(rng));
    T.topRightCorner<3, 1>() = Eigen::Vector3d(u_lat(rng), u_lat(rng), u_z(rng));
    const Eigen::MatrixX3d P_c = transform_points(T, P_o);
    if ((P_c.col(2).array() <= 1e-6).any()) {
      continue;                             // reject and redraw
    }
    out.push_back(P_c);
  }
  return out;
}

double percentile(std::vector<double> v, double q)
{
  return blindspot::percentile_linear(v, q);
}

double calibrate_area(
  const Eigen::MatrixX3d & P_o, double pct, int n, unsigned int seed,
  std::pair<double, double> z_range, double lateral, double tilt = 0.3)
{
  std::vector<double> vals;
  for (const auto & P_c : healthy_poses(P_o, n, seed, z_range, lateral, tilt)) {
    vals.push_back(blindspot::polygon_sigma(project(P_c)));
  }
  return percentile(vals, pct);
}

double calibrate_sigma6(
  const Eigen::MatrixX3d & P_o, double pct, int n, unsigned int seed,
  std::pair<double, double> z_range, double lateral, double tilt = 0.3)
{
  std::vector<double> vals;
  for (const auto & P_c : healthy_poses(P_o, n, seed, z_range, lateral, tilt)) {
    vals.push_back(sigma_6(interaction_matrix(project(P_c), P_c.col(2))));
  }
  return percentile(vals, pct);
}

// ---------------------------------------------------------------------------
// the act
// ---------------------------------------------------------------------------

const double kPhi = 11.0 * M_PI / 180.0;   // marker layout rotation on the panel
constexpr double kRT = 0.05;               // ring radius, metres
constexpr double kRS = 0.006;              // marker sphere radius, metres
const double kFx = (640.0 / 2.0) / std::tan(1.0472 / 2.0);  // the URDF wrist camera
const double kXL = (640.0 / 2.0) / kFx;
const double kYL = (480.0 / 2.0) / kFx;

using blindspot::arm_law::kDt;
using blindspot::arm_law::kZDesired;

Eigen::MatrixX3d hex22(double r)
{
  return blindspot::arm_law::hexagon(r, kPhi);
}

/// Both flaps fold AWAY from the camera about the hinge line (y = 0).
Eigen::MatrixX3d fold22(double theta)
{
  Eigen::MatrixX3d P = hex22(kRT);
  const Eigen::VectorXd y = P.col(1);
  P.col(1) = y * std::cos(theta);
  P.col(2) = y.cwiseAbs() * std::sin(theta);
  return P;
}

enum class Rule { kPartition, kIbvs, kGuard };

struct Outcome
{
  double standoff{9.0};
  int lost{0};
  int fired{0};
  std::optional<double> first{};
  double min_area{9.0};
  double min_s6{9.0};
  double peak{0.0};
};

/// Converged at the goal, then the part folds. Free camera, exact dots.
Outcome act(
  Rule rule, double area_thresh, double s6_thresh, double fold_deg = 85.0,
  double fold_s = 1.0, double t_fold = 1.0, double T = 6.0)
{
  Eigen::Matrix4d cTo = Eigen::Matrix4d::Identity();
  cTo(2, 3) = kZDesired;
  const Eigen::MatrixX3d hex_goal = hex22(kRT / kZDesired);
  const Eigen::MatrixX2d s_star0 =
    blindspot::arm_law::order_by_angle(hex_goal.leftCols<2>());

  Outcome o;
  const int steps = static_cast<int>(T / kDt);
  for (int k = 0; k < steps; ++k) {
    const double t = k * kDt;
    const double ramp = std::clamp((t - t_fold) / fold_s, 0.0, 1.0);
    const Eigen::MatrixX3d P = fold22(fold_deg * M_PI / 180.0 * ramp);
    const Eigen::MatrixX3d Pc = transform_points(cTo, P);
    if ((Pc.col(2).array() <= 0.02).any()) {
      ++o.lost;
      break;
    }
    const Eigen::MatrixX2d s_all = project(Pc);

    bool all_visible = true;
    for (Eigen::Index i = 0; i < s_all.rows(); ++i) {
      if (std::abs(s_all(i, 0)) >= kXL || std::abs(s_all(i, 1)) >= kYL) {
        all_visible = false;
        break;
      }
    }
    bool merged = false;
    for (Eigen::Index i = 0; i < 6 && !merged; ++i) {
      for (Eigen::Index j = i + 1; j < 6; ++j) {
        const double rad_i = kRS / Pc(i, 2);
        const double rad_j = kRS / Pc(j, 2);
        if ((s_all.row(i) - s_all.row(j)).norm() < rad_i + rad_j + 3.0 / kFx) {
          merged = true;
          break;
        }
      }
    }

    Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
    if (all_visible && !merged) {
      const Eigen::MatrixX2d s = blindspot::arm_law::order_by_angle(s_all);
      const double m_area = blindspot::polygon_sigma(s) / area_thresh;
      o.min_area = std::min(o.min_area, m_area);
      o.min_s6 = std::min(
        o.min_s6, sigma_6(interaction_matrix(s_all, Pc.col(2))) / s6_thresh);

      bool part = true;
      if (rule == Rule::kIbvs) {
        part = false;
      } else if (rule == Rule::kGuard) {
        part = m_area > 1.0;
      }
      if (rule == Rule::kGuard && !part) {
        ++o.fired;
        if (!o.first.has_value()) {
          o.first = std::round((t - t_fold) * 100.0) / 100.0;
        }
      }
      v = blindspot::arm_law::ibvs_twist(
        s, blindspot::arm_law::best_cyclic_match(s, s_star0), part);
      o.peak = std::max(o.peak, v.norm());
    } else {
      ++o.lost;
    }
    cTo = se3_exp(-v * kDt) * cTo;
    o.standoff = std::min(o.standoff, -cTo.inverse()(2, 3));
  }
  return o;
}

}  // namespace

int main()
{
  std::printf("--- the folding part: the arm act in Eigen ---\n");

  // Same pose family the demo node calibrates on: it logged 2.009e-01 for N=6.
  const double A22 = calibrate_area(hex22(kRT), 1.0, 4000, 0, {0.18, 0.40}, 0.05);
  const double S22 = calibrate_sigma6(hex22(kRT), 1.0, 4000, 0, {0.18, 0.40}, 0.05);
  {
    char d[128];
    std::snprintf(
      d, sizeof(d), "(%.4e here, 2.009e-01 logged in Gazebo; different RNG stream)", A22);
    check(
      "the demo's guard threshold reproduces from the study's calibrator",
      std::abs(A22 - 2.009e-01) / 2.009e-01 < 0.02, d);
  }

  const Outcome a_part = act(Rule::kPartition, A22, S22);
  const Outcome a_guard = act(Rule::kGuard, A22, S22);
  const Outcome a_ibvs = act(Rule::kIbvs, A22, S22);

  {
    char d[160];
    std::snprintf(
      d, sizeof(d),
      "(%.3f m from a 0.26 m standoff, %d steps without all six markers)",
      a_part.standoff, a_part.lost);
    check(
      "85 deg fold in 1 s: the partition drives into the part and loses it",
      a_part.standoff < 0.10 && a_part.lost > 10, d);
  }
  {
    char d[128];
    std::snprintf(
      d, sizeof(d), "(fired %.1f s into the fold, %.3f m)",
      a_guard.first.value_or(-1.0), a_guard.standoff);
    check(
      "  the guard fires during the fold and keeps its standoff",
      a_guard.first.has_value() && a_guard.first.value() <= 1.0 &&
      a_guard.standoff > 0.15 && a_guard.lost == 0, d);
  }
  {
    char d[64];
    std::snprintf(d, sizeof(d), "(%.3f m)", a_ibvs.standoff);
    check(
      "  plain IBVS alone holds it too: the partition is what fails",
      a_ibvs.standoff > 0.20 && a_ibvs.lost == 0, d);
  }
  {
    char d[128];
    std::snprintf(
      d, sizeof(d), "(min sigma_6 margin %.2fx, area fell to %.2fx)",
      a_part.min_s6, a_part.min_area);
    check(
      "  sigma_6 never fires: folding AWAY adds depth, so L stays conditioned",
      a_part.min_s6 > 1.2, d);
  }

  // The guard watches a quantity its own controller regulates: when the fold
  // is slow enough, the lunge restores the area before it crosses the
  // threshold.
  //
  // Do NOT assert the absolute depth this lunge reaches. It carries the
  // camera through a near-degenerate pose where the truncated pseudo-inverse
  // is extremely sensitive to rounding, so the depth is not stable across
  // BLAS implementations. The claim is that the guarded and unguarded runs
  // are the SAME run, which is the 1e-9 agreement asserted below.
  const Outcome s_part = act(Rule::kPartition, A22, S22, 85.0, 3.0, 1.0, 9.0);
  const Outcome s_guard = act(Rule::kGuard, A22, S22, 85.0, 3.0, 1.0, 9.0);
  {
    char d[160];
    std::snprintf(
      d, sizeof(d), "(85 deg over 3 s: area never below %.2fx, both reach %.3f m)",
      s_part.min_area, s_guard.standoff);
    check(
      "a SLOW fold is masked: the lunge restores the area, the guard misses",
      s_guard.fired == 0 && std::abs(s_guard.standoff - s_part.standoff) < 1e-9, d);
  }

  const auto fires = [&](double deg, double sec) {
      return act(Rule::kGuard, A22, S22, deg, sec, 1.0, 4.0 + 2.0 * sec).fired > 0;
    };
  check(
    "the window is a race between the fold and the depth loop",
    fires(75, 1.0) && !fires(75, 1.5) && fires(80, 1.5) && !fires(80, 2.0) &&
    fires(85, 2.0) && !fires(85, 3.0),
    "(fires: 75 deg <= 1 s, 80 deg <= 1.5 s, 85 deg <= 2 s)");

  std::printf("\n%d/%d passed\n", g_pass, g_total);
  return g_pass == g_total ? 0 : 1;
}
