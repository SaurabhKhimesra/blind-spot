#include "blindspot/ros_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace blindspot
{

std::string to_string(Ordering how)
{
  return how == Ordering::kById ? "id" : "angular";
}

std::string to_string(IntrinsicsSource src)
{
  switch (src) {
    case IntrinsicsSource::kCameraInfo: return "camera_info";
    case IntrinsicsSource::kFovParameter: return "fovy_deg parameter";
    default: return "uncalibrated";
  }
}

std::vector<int> angular_order(const Eigen::Ref<const Eigen::MatrixX2d> & uv)
{
  const Eigen::Index n = uv.rows();
  const Eigen::RowVector2d c = uv.colwise().mean();
  std::vector<double> angle(static_cast<size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i) {
    angle[static_cast<size_t>(i)] = std::atan2(uv(i, 1) - c(1), uv(i, 0) - c(0));
  }
  std::vector<int> idx(static_cast<size_t>(n));
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(
    idx.begin(), idx.end(),
    [&angle](int a, int b) {return angle[static_cast<size_t>(a)] <
      angle[static_cast<size_t>(b)];});
  return idx;
}

namespace
{

/// Parses an id as a number. Ids that parse sort numerically and ahead of
/// any that do not.
bool numeric_id(const std::string & s, double * out)
{
  try {
    size_t used = 0;
    const double v = std::stod(s, &used);
    if (used != s.size()) {
      return false;
    }
    *out = v;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

Eigen::MatrixX2d take_rows(
  const Eigen::Ref<const Eigen::MatrixX2d> & uv, const std::vector<int> & idx)
{
  Eigen::MatrixX2d out(static_cast<Eigen::Index>(idx.size()), 2);
  for (size_t i = 0; i < idx.size(); ++i) {
    out.row(static_cast<Eigen::Index>(i)) = uv.row(idx[i]);
  }
  return out;
}

double shoelace(const Eigen::Ref<const Eigen::MatrixX2d> & p)
{
  const Eigen::Index n = p.rows();
  double cross = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Eigen::Index j = (i + 1) % n;
    cross += p(i, 0) * p(j, 1) - p(i, 1) * p(j, 0);
  }
  return 0.5 * std::abs(cross);
}

}  // namespace

OrderedFeatures order_features(
  const Eigen::Ref<const Eigen::MatrixX2d> & uv, const std::vector<std::string> & ids)
{
  const size_t n = static_cast<size_t>(uv.rows());
  if (!ids.empty() && ids.size() == n) {
    std::vector<int> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    std::stable_sort(
      idx.begin(), idx.end(),
      [&ids](int a, int b) {
        double va = 0.0;
        double vb = 0.0;
        const bool na = numeric_id(ids[static_cast<size_t>(a)], &va);
        const bool nb = numeric_id(ids[static_cast<size_t>(b)], &vb);
        if (na != nb) {
          return na;                        // numeric ids sort first
        }
        if (na) {
          return va < vb;
        }
        return ids[static_cast<size_t>(a)] < ids[static_cast<size_t>(b)];
      });
    std::vector<std::string> ordered_ids;
    ordered_ids.reserve(n);
    for (const int i : idx) {
      ordered_ids.push_back(ids[static_cast<size_t>(i)]);
    }
    return OrderedFeatures{take_rows(uv, idx), std::move(ordered_ids), Ordering::kById};
  }
  return OrderedFeatures{take_rows(uv, angular_order(uv)), {}, Ordering::kAngular};
}

CameraInfoIntrinsics intrinsics_from_camera_info(
  const std::array<double, 9> & k, int width, int height)
{
  const double fx = k[0];
  const double fy = k[4];
  const double cx = k[2];
  const double cy = k[5];
  if (fx > 0.0 && fy > 0.0) {
    return CameraInfoIntrinsics{
      Intrinsics{fx, fy, cx, cy}, IntrinsicsSource::kCameraInfo, true};
  }
  // Nothing usable was published. Only the fallback principal point is
  // meaningful; the caller must supply a field of view.
  return CameraInfoIntrinsics{
    Intrinsics{0.0, 0.0, (width - 1) / 2.0, (height - 1) / 2.0},
    IntrinsicsSource::kUncalibrated, false};
}

NormalisedFeatures features_to_normalised(
  const Eigen::Ref<const Eigen::MatrixX2d> & uv,
  const Intrinsics & k,
  const std::vector<std::string> & ids)
{
  OrderedFeatures ordered = order_features(uv, ids);
  return NormalisedFeatures{
    pixels_to_normalised(ordered.uv, k), std::move(ordered.ids), ordered.how};
}

bool ordering_is_suspect(const Eigen::Ref<const Eigen::MatrixX2d> & uv, double tol)
{
  if (uv.rows() < 3) {
    return false;
  }
  // Not the convex hull: the angular sort only coincides with it when the
  // points are in convex position.
  const Eigen::MatrixX2d sorted = take_rows(uv, angular_order(uv));
  const double reference = shoelace(sorted);
  if (reference <= 0.0) {
    return false;
  }
  return shoelace(uv) < tol * reference;
}

}  // namespace blindspot
