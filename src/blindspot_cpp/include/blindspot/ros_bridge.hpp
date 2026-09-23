// Turning detector output into something the guard can score.
//
// No ROS includes here on purpose: this is the part that can be wrong, so it
// is testable without a ROS installation. The node is a thin wrapper, and
// reading a Detection2D lives in detection_msg.hpp.
//
// Two things this exists to get right.
//
// ORDERING. The guard signal is a shoelace polygon area, which depends on the
// ORDER of the points. A detector reports whatever order it happens to find,
// and an order that crosses itself gives a small area - firing the guard for
// a reason that has nothing to do with the geometry. Points are therefore
// ordered by detection id, which reproduces the order the calibration used.
// Without ids we fall back to an angular sort about the centroid, which is
// correct for a convex arrangement and is reported so it can be seen in the
// diagnostic.
//
// UNITS. The guard works in normalised image coordinates. Use the principal
// point from CameraInfo when the camera is actually calibrated; only fall
// back to (W-1)/2 when it is not, and say which was used. Note (W-1)/2, not
// W/2: OpenGL maps NDC onto a continuous pixel range so the axis lands at
// W/2, while OpenCV indexes pixel centres half a pixel lower.
#ifndef BLINDSPOT__ROS_BRIDGE_HPP_
#define BLINDSPOT__ROS_BRIDGE_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "blindspot/units.hpp"

namespace blindspot
{

/// How the features were put in order, for the diagnostic.
enum class Ordering { kById, kAngular };

std::string to_string(Ordering how);

/// Where the intrinsics came from, for the diagnostic.
enum class IntrinsicsSource { kCameraInfo, kUncalibrated, kFovParameter };

std::string to_string(IntrinsicsSource src);

/// Indices that sort points counter-clockwise about their centroid.
std::vector<int> angular_order(const Eigen::Ref<const Eigen::MatrixX2d> & uv);

struct OrderedFeatures
{
  Eigen::MatrixX2d uv;
  std::vector<std::string> ids;   ///< empty when ordered angularly
  Ordering how{Ordering::kAngular};
};

/// Order detections so the shoelace area means what it should.
///
/// Ids that parse as numbers sort numerically and ahead of any that do not.
/// Pass an empty `ids` to force the angular fallback.
OrderedFeatures order_features(
  const Eigen::Ref<const Eigen::MatrixX2d> & uv,
  const std::vector<std::string> & ids = {});

struct CameraInfoIntrinsics
{
  Intrinsics k;
  IntrinsicsSource source{IntrinsicsSource::kUncalibrated};
  /// false when the camera published zeros, in which case only cx, cy in `k`
  /// are meaningful and the caller must supply a field of view instead.
  bool usable{false};
};

/// Intrinsics from a CameraInfo K matrix.
///
/// K is the row-major 9-vector [fx 0 cx; 0 fy cy; 0 0 1]. An uncalibrated
/// camera publishes zeros, in which case there is nothing to use.
CameraInfoIntrinsics intrinsics_from_camera_info(
  const std::array<double, 9> & k, int width, int height);

struct NormalisedFeatures
{
  Eigen::MatrixX2d s;
  std::vector<std::string> ids;
  Ordering how{Ordering::kAngular};
};

/// Detector pixels -> ordered normalised coordinates for the guard.
NormalisedFeatures features_to_normalised(
  const Eigen::Ref<const Eigen::MatrixX2d> & uv,
  const Intrinsics & k,
  const std::vector<std::string> & ids = {});

/// True if the given order encloses much less area than an angular sort.
///
/// A cheap self-check: for points in a sane order the two agree. If they do
/// not, the order crosses itself and the area - and therefore the guard
/// decision - is not measuring the geometry.
///
/// The reference is the polygon of the angular sort about the centroid, which
/// is the convex hull only when the points are in convex position; for a
/// concave arrangement it under-reports, so this errs towards staying quiet.
bool ordering_is_suspect(const Eigen::Ref<const Eigen::MatrixX2d> & uv, double tol = 0.98);

}  // namespace blindspot

#endif  // BLINDSPOT__ROS_BRIDGE_HPP_
