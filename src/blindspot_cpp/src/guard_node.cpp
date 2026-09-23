// Publishes, per detection frame, whether the 2001 partition is safe now.
//
//     ros2 run blindspot_cpp guard_node --ros-args
//         -p calibration:=/abs/path/my_target.json
//         -r detections:=/aruco/detections
//         -r camera_info:=/camera/camera_info
//
// Subscribes
//     detections   vision_msgs/Detection2DArray  (feature points, ideally with ids)
//     camera_info  sensor_msgs/CameraInfo        (for the principal point)
//
// Publishes
//     ~/partition_ok  std_msgs/Bool              latch this in your servo loop
//     /diagnostics    diagnostic_msgs/DiagnosticArray
//
// This node does NOT control anything. Keep your controller; gate it on the
// Bool:
//
//     if (partition_ok) { v = my_partitioned_control(...); }
//     else              { v = my_plain_control(...); }
//
// There is no default threshold. `calibration` is required and must come from
// a KNOWN-GOOD target, because a threshold calibrated on a degenerate one
// never fires and nothing looks wrong.
#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "blindspot/detection_msg.hpp"
#include "blindspot/guard.hpp"
#include "blindspot/ros_bridge.hpp"
#include "blindspot/units.hpp"

namespace
{

using blindspot::detection_center;
using blindspot::detection_label;
using diagnostic_msgs::msg::DiagnosticStatus;

class GuardNode : public rclcpp::Node
{
public:
  GuardNode()
  : Node("blindspot_guard")
  {
    declare_parameter<std::string>("calibration", "");
    declare_parameter<double>("fovy_deg", 0.0);
    declare_parameter<std::vector<std::string>>("expected_ids", std::vector<std::string>{});

    const auto path = get_parameter("calibration").as_string();
    if (path.empty()) {
      throw std::runtime_error(
              "parameter 'calibration' is required; this node ships no default "
              "threshold. Calibrate on a KNOWN-GOOD target:\n"
              "  ros2 run blindspot_cpp calibrate --geometry target.json "
              "--goal-pose pose.json -o my_target.json");
    }
    guard_ = std::make_unique<blindspot::FeatureGuard>(blindspot::FeatureGuard::load(path));

    std::string table;
    for (const auto & [n, v] : guard_->calibration().thresholds()) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%d: %.3e", n, v);
      table += (table.empty() ? "" : ", ") + std::string(buf);
    }
    RCLCPP_INFO(
      get_logger(), "loaded %s, thresholds by visible count: {%s}", path.c_str(),
      table.c_str());

    fovy_ = get_parameter("fovy_deg").as_double();
    expected_ = get_parameter("expected_ids").as_string_array();

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", latched,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr m) {
        k_ = m->k;
        width_ = static_cast<int>(m->width);
        height_ = static_cast<int>(m->height);
        have_info_ = true;
      });
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "detections", 10,
      [this](vision_msgs::msg::Detection2DArray::SharedPtr m) {on_detections(*m);});

    pub_ok_ = create_publisher<std_msgs::msg::Bool>("~/partition_ok", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  }

private:
  struct Resolved
  {
    blindspot::Intrinsics k;
    std::string source;
    bool ok{false};
  };

  Resolved intrinsics()
  {
    if (!have_info_) {
      return Resolved{};
    }
    std::array<double, 9> k{};
    for (size_t i = 0; i < 9; ++i) {
      k[i] = k_[i];
    }
    auto from_info = blindspot::intrinsics_from_camera_info(k, width_, height_);
    if (from_info.usable) {
      return Resolved{from_info.k, blindspot::to_string(from_info.source), true};
    }
    if (fovy_ <= 0.0) {
      return Resolved{};
    }
    if (!warned_uncal_) {
      RCLCPP_WARN(
        get_logger(),
        "camera_info carries no intrinsics; using fovy_deg=%.1f with principal "
        "point (W-1)/2", fovy_);
      warned_uncal_ = true;
    }
    return Resolved{
      blindspot::intrinsics_from_fov(width_, height_, fovy_),
      blindspot::to_string(blindspot::IntrinsicsSource::kFovParameter), true};
  }

  void on_detections(const vision_msgs::msg::Detection2DArray & msg)
  {
    const Resolved intr = intrinsics();
    if (!intr.ok) {
      publish_diag(
        DiagnosticStatus::ERROR, "waiting for camera_info (or set fovy_deg)", {});
      return;
    }

    std::vector<std::pair<double, double>> uv_list;
    std::vector<std::string> ids;
    bool have_ids = true;
    for (const auto & det : msg.detections) {
      const std::string did = detection_label(det);
      if (!expected_.empty() &&
        std::find(expected_.begin(), expected_.end(), did) == expected_.end())
      {
        continue;
      }
      uv_list.push_back(detection_center(det));
      ids.push_back(did);
      if (did.empty()) {
        have_ids = false;
      }
    }
    if (uv_list.empty()) {
      publish(false, DiagnosticStatus::WARN, "no features detected", {{"n_features", "0"}});
      return;
    }

    Eigen::MatrixX2d uv(static_cast<Eigen::Index>(uv_list.size()), 2);
    for (size_t i = 0; i < uv_list.size(); ++i) {
      uv(static_cast<Eigen::Index>(i), 0) = uv_list[i].first;
      uv(static_cast<Eigen::Index>(i), 1) = uv_list[i].second;
    }

    const std::vector<std::string> ids_or_none = have_ids ? ids : std::vector<std::string>{};
    const auto norm = blindspot::features_to_normalised(uv, intr.k, ids_or_none);
    const auto ordered = blindspot::order_features(uv, ids_or_none);
    const bool suspect = blindspot::ordering_is_suspect(ordered.uv);

    const auto r = guard_->evaluate(norm.s);
    auto level = r.decision ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
    std::string text = r.decision ? "partition ok"
      : "partition DROPPED - use your plain control law";
    if (suspect) {
      level = DiagnosticStatus::ERROR;
      text = "feature ordering crosses itself, so the area is not measuring the "
        "geometry; the decision is unreliable";
    }

    char sig[32];
    char thr[32];
    char mar[32];
    std::snprintf(sig, sizeof(sig), "%.6e", r.signal);
    std::snprintf(thr, sizeof(thr), "%.6e", r.threshold);
    std::snprintf(mar, sizeof(mar), "%.3f", r.margin);
    publish(
      r.decision, level, text, {
            {"signal", sig},
            {"threshold", thr},
            {"margin", mar},
            {"n_features", std::to_string(r.n_features)},
            {"decision", r.decision ? "partition_ok" : "drop_partition"},
            {"ordering", blindspot::to_string(norm.how)},
            {"intrinsics", intr.source}});
  }

  void publish(
    bool ok, uint8_t level, const std::string & text,
    const std::vector<std::pair<std::string, std::string>> & kvs)
  {
    std_msgs::msg::Bool b;
    b.data = ok;
    pub_ok_->publish(b);
    publish_diag(level, text, kvs);
  }

  void publish_diag(
    uint8_t level, const std::string & text,
    const std::vector<std::pair<std::string, std::string>> & kvs)
  {
    DiagnosticStatus st;
    st.level = level;
    st.name = "blindspot: partition guard";
    st.hardware_id = "blindspot";
    st.message = text;
    for (const auto & [k, v] : kvs) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
    }
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    pub_diag_->publish(arr);
  }

  std::unique_ptr<blindspot::FeatureGuard> guard_;
  double fovy_{0.0};
  std::vector<std::string> expected_;
  std::array<double, 9> k_{};
  int width_{0};
  int height_{0};
  bool have_info_{false};
  bool warned_uncal_{false};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ok_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<GuardNode> node;
  try {
    node = std::make_shared<GuardNode>();
  } catch (const std::exception & e) {
    // A missing or unreadable calibration is the expected way to get here,
    // and the message says how to make one. Exit non-zero rather than run
    // without a threshold.
    RCLCPP_ERROR(rclcpp::get_logger("blindspot_guard"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
