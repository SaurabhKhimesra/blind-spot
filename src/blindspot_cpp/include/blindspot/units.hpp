// Image coordinate conventions.
//
// The guard works in NORMALISED image coordinates: x = X/Z, y = Y/Z, with the
// camera looking along +z, x to the right and y DOWN. That is the computer
// vision convention, not the OpenGL one.
//
// Detectors report pixels, so convert before calling the guard.
//
// The principal point is (W-1)/2, not W/2. A renderer maps normalised device
// coordinates onto a CONTINUOUS pixel range [0, W], so the optical axis lands
// at continuous coordinate W/2; OpenCV indexes pixel CENTRES, which sit half a
// pixel lower. Using W/2 shows up as a systematic half-pixel bias in every
// feature, which is easy to miss: here it hid for a while behind a corner
// refinement whose own offset happened to cancel most of it.
#ifndef BLINDSPOT__UNITS_HPP_
#define BLINDSPOT__UNITS_HPP_

#include <Eigen/Core>

namespace blindspot
{

/// Camera intrinsics in pixels. cx, cy are (W-1)/2 and (H-1)/2 for an ideal
/// camera, never W/2 and H/2.
struct Intrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

/// (fx, fy, cx, cy) for a camera specified by vertical field of view.
Intrinsics intrinsics_from_fov(int width, int height, double fovy_deg);

/// Pixel coordinates -> normalised image coordinates.
///
/// uv : (N, 2) pixel coordinates, origin at the centre of the top-left pixel
///      (the OpenCV convention).
/// k  : intrinsics. Use cx = (W-1)/2 and cy = (H-1)/2 unless you have a
///      calibrated principal point.
Eigen::MatrixX2d pixels_to_normalised(const Eigen::MatrixX2d & uv, const Intrinsics & k);

}  // namespace blindspot

#endif  // BLINDSPOT__UNITS_HPP_
