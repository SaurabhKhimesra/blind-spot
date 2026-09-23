#include "blindspot/units.hpp"

#include <cmath>
#include <stdexcept>

namespace blindspot
{

Intrinsics intrinsics_from_fov(int width, int height, double fovy_deg)
{
  const double fy = (height / 2.0) / std::tan((fovy_deg * M_PI / 180.0) / 2.0);
  // fx == fy: square pixels. Principal point is (W-1)/2, not W/2.
  return Intrinsics{fy, fy, (width - 1) / 2.0, (height - 1) / 2.0};
}

Eigen::MatrixX2d pixels_to_normalised(const Eigen::MatrixX2d & uv, const Intrinsics & k)
{
  if (k.fx == 0.0 || k.fy == 0.0) {
    throw std::invalid_argument(
            "fx and fy are required and must be non-zero; for an ideal camera use "
            "intrinsics_from_fov(), whose principal point is (W-1)/2 and (H-1)/2, "
            "not W/2 and H/2");
  }
  Eigen::MatrixX2d s(uv.rows(), 2);
  s.col(0) = (uv.col(0).array() - k.cx) / k.fx;
  s.col(1) = (uv.col(1).array() - k.cy) / k.fy;
  return s;
}

}  // namespace blindspot
