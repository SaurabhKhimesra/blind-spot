// Blob detection for the panel markers. The only part that needs OpenCV.
//
// Kept apart from the control law (arm_law.hpp) so that the law stays
// Eigen-only and the checks can include it without OpenCV.
#ifndef BLINDSPOT__DETECT_HPP_
#define BLINDSPOT__DETECT_HPP_

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace blindspot
{

/// Marker centres in pixels, (N, 2). `rgb` is 8-bit 3-channel, RGB order.
///
/// The panel carries exactly six dots. Rendering artefacts and the post edge
/// can add spurious components, so keep six: a 7th blob otherwise breaks
/// correspondence against the six desired features. The six dots are
/// identical circles, so pick the six most SIMILAR blobs, not the six
/// largest. Taking the largest lets the robot's own dark wrist - which is in
/// frame at close standoff - masquerade as a fiducial and displace a real
/// dot. Offline the control law converges to |e| 0.0008 on ground-truth
/// features, so divergence in the sim was perception, not control.
Eigen::MatrixX2d detect_dots(const cv::Mat & rgb);

}  // namespace blindspot

#endif  // BLINDSPOT__DETECT_HPP_
