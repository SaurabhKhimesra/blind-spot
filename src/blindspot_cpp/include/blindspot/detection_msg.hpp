// Reading a vision_msgs/Detection2D. Split out from the node so the checks
// can exercise it directly.
#ifndef BLINDSPOT__DETECTION_MSG_HPP_
#define BLINDSPOT__DETECTION_MSG_HPP_

#include <string>
#include <utility>

#include <vision_msgs/msg/detection2_d.hpp>

namespace blindspot
{

/// (x, y) pixel centre. BoundingBox2D.center is a vision_msgs/Pose2D, which
/// carries a `position`.
inline std::pair<double, double> detection_center(const vision_msgs::msg::Detection2D & det)
{
  return {det.bbox.center.position.x, det.bbox.center.position.y};
}

/// Stable id, or empty if the detector reports none. Prefers Detection2D.id,
/// falls back to the first hypothesis class_id.
inline std::string detection_label(const vision_msgs::msg::Detection2D & det)
{
  if (!det.id.empty()) {
    return det.id;
  }
  for (const auto & r : det.results) {
    if (!r.hypothesis.class_id.empty()) {
      return r.hypothesis.class_id;
    }
  }
  return {};
}

}  // namespace blindspot

#endif  // BLINDSPOT__DETECTION_MSG_HPP_
