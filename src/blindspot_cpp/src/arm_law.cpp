#include "blindspot/arm_law.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/SVD>

namespace blindspot
{
namespace arm_law
{

namespace
{

/// Feature error, flattened ROW-major: [dx0, dy0, dx1, dy1, ...].
///
/// Eigen stores column-major by default, so a plain reshape here would
/// interleave the error the wrong way and silently permute the rows of L.
Eigen::VectorXd flatten_error(
  const Eigen::Ref<const Eigen::MatrixX2d> & s,
  const Eigen::Ref<const Eigen::MatrixX2d> & s_star)
{
  Eigen::VectorXd e(2 * s.rows());
  for (Eigen::Index i = 0; i < s.rows(); ++i) {
    e(2 * i) = s(i, 0) - s_star(i, 0);
    e(2 * i + 1) = s(i, 1) - s_star(i, 1);
  }
  return e;
}

double wrap(double a)
{
  // Floored modulo, so the result lands in [-pi, pi) for negative a too.
  // std::fmod alone truncates toward zero and would not.
  const double two_pi = 2.0 * M_PI;
  double m = std::fmod(a + M_PI, two_pi);
  if (m < 0.0) {
    m += two_pi;
  }
  return m - M_PI;
}

double line_angle(const Eigen::Ref<const Eigen::MatrixX2d> & s)
{
  return std::atan2(s(1, 1) - s(0, 1), s(1, 0) - s(0, 0));
}

}  // namespace

Intrinsics default_intrinsics()
{
  const double fx = (kImgW / 2.0) / std::tan(kHFov / 2.0);
  return Intrinsics{fx, fx, (kImgW - 1) / 2.0, (kImgH - 1) / 2.0};
}

Eigen::MatrixX3d hexagon(double r, double phi)
{
  Eigen::MatrixX3d p(6, 3);
  for (int i = 0; i < 6; ++i) {
    const double a = 2.0 * M_PI * i / 6.0 + phi;
    p(i, 0) = r * std::cos(a);
    p(i, 1) = r * std::sin(a);
    p(i, 2) = 0.0;
  }
  return p;
}

Eigen::MatrixX2d order_by_angle(const Eigen::Ref<const Eigen::MatrixX2d> & pts)
{
  const Eigen::Index n = pts.rows();
  const Eigen::RowVector2d c = pts.colwise().mean();
  std::vector<double> angle(static_cast<size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i) {
    angle[static_cast<size_t>(i)] = std::atan2(pts(i, 1) - c(1), pts(i, 0) - c(0));
  }
  std::vector<int> idx(static_cast<size_t>(n));
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(
    idx.begin(), idx.end(),
    [&angle](int a, int b) {return angle[static_cast<size_t>(a)] <
      angle[static_cast<size_t>(b)];});
  Eigen::MatrixX2d out(n, 2);
  for (Eigen::Index i = 0; i < n; ++i) {
    out.row(i) = pts.row(idx[static_cast<size_t>(i)]);
  }
  return out;
}

Eigen::MatrixX2d best_cyclic_match(
  const Eigen::Ref<const Eigen::MatrixX2d> & s,
  const Eigen::Ref<const Eigen::MatrixX2d> & s_star)
{
  const Eigen::Index n = s_star.rows();
  Eigen::MatrixX2d best = s_star;
  double best_err = std::numeric_limits<double>::infinity();
  for (Eigen::Index k = 0; k < n; ++k) {
    // cand[i] = s_star[(i - k) mod n]: rotate the goal by k places.
    Eigen::MatrixX2d cand(n, 2);
    for (Eigen::Index i = 0; i < n; ++i) {
      cand.row(i) = s_star.row(((i - k) % n + n) % n);
    }
    const double err = (s - cand).norm();
    if (err < best_err) {
      best = cand;
      best_err = err;
    }
  }
  return best;
}

Eigen::MatrixXd interaction(const Eigen::Ref<const Eigen::MatrixX2d> & s, double Z)
{
  const Eigen::Index n = s.rows();
  Eigen::MatrixXd L(2 * n, 6);
  for (Eigen::Index i = 0; i < n; ++i) {
    const double x = s(i, 0);
    const double y = s(i, 1);
    L.row(2 * i) << -1.0 / Z, 0.0, x / Z, x * y, -(1.0 + x * x), y;
    L.row(2 * i + 1) << 0.0, -1.0 / Z, y / Z, 1.0 + y * y, -x * y, -x;
  }
  return L;
}

Eigen::MatrixXd pinv_trunc(const Eigen::Ref<const Eigen::MatrixXd> & L, double tau)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(L, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd & S = svd.singularValues();
  Eigen::VectorXd Si(S.size());
  for (Eigen::Index i = 0; i < S.size(); ++i) {
    // Directions below tau * s_max are dropped, not merely damped, and there
    // is no fallback that forces the first direction back in - that belongs
    // to the study's reference controller, not to this law.
    Si(i) = (S(i) > tau * S(0)) ? (S(i) > 0.0 ? 1.0 / S(i) : 1.0) : 0.0;
  }
  return svd.matrixV() * Si.asDiagonal() * svd.matrixU().transpose();
}

double poly_sigma(const Eigen::Ref<const Eigen::MatrixX2d> & s)
{
  // NOTE: this floors the AREA at 1e-12 before the square root, so it never
  // returns exactly zero. blindspot::polygon_sigma (guard.hpp) floors at 0
  // and does return zero. They are different quantities used for different
  // jobs - a control input here, a monitored signal there - so do not report
  // one as the other.
  const Eigen::Index n = s.rows();
  double cross = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Eigen::Index j = (i + 1) % n;
    cross += s(i, 0) * s(j, 1) - s(i, 1) * s(j, 0);
  }
  return std::sqrt(std::max(0.5 * std::abs(cross), 1e-12));
}

Eigen::Matrix<double, 6, 1> ibvs_twist(
  const Eigen::Ref<const Eigen::MatrixX2d> & s,
  const Eigen::Ref<const Eigen::MatrixX2d> & s_star,
  bool use_partition)
{
  const Eigen::VectorXd e = flatten_error(s, s_star);
  const Eigen::MatrixXd L = interaction(s, kZDesired);

  if (use_partition && s.rows() >= 3) {
    const double sig = poly_sigma(s);
    const double sig_star = poly_sigma(s_star);
    const double vz = 0.6 * std::log(sig_star / std::max(sig, 1e-9));
    const double wz = 0.6 * wrap(line_angle(s) - line_angle(s_star));

    // vx, vy, wx, wy come from the reduced inverse; vz and wz are driven
    // straight from the image measurements above, with their coupling term
    // ADDED to the error rather than subtracted.
    const std::array<int, 4> xy{0, 1, 3, 4};
    const std::array<int, 2> z{2, 5};
    Eigen::MatrixXd L_xy(L.rows(), 4);
    for (int c = 0; c < 4; ++c) {
      L_xy.col(c) = L.col(xy[static_cast<size_t>(c)]);
    }
    Eigen::MatrixXd L_z(L.rows(), 2);
    for (int c = 0; c < 2; ++c) {
      L_z.col(c) = L.col(z[static_cast<size_t>(c)]);
    }
    Eigen::Vector2d vz_wz(vz, wz);
    const Eigen::VectorXd v_xy = -pinv_trunc(L_xy, kTau) * (kLam * e + L_z * vz_wz);

    Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
    for (int c = 0; c < 4; ++c) {
      v(xy[static_cast<size_t>(c)]) = v_xy(c);
    }
    v(2) = vz;
    v(5) = wz;
    return v;
  }
  return -kLam * (pinv_trunc(L, kTau) * e);
}

}  // namespace arm_law
}  // namespace blindspot
