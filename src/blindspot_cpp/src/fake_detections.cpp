// Synthetic detections, so the guard node can be exercised without a camera.
//
// Walks a virtual camera through three regimes and publishes the resulting
// marker centres, which is enough to see the guard change its mind:
//
//     healthy    all 6 markers on the ring          -> partition ok
//     occluded   only 2 markers survive             -> partition dropped
//     collapsed  all 6 markers, target near a line  -> partition dropped
//
//     ros2 run blindspot_cpp fake_detections
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "blindspot/units.hpp"

namespace
{

constexpr int WIDTH = 1920;
constexpr int HEIGHT = 1440;
constexpr double FOVY = 45.0;
constexpr int PHASE = 40;               // frames per regime

Eigen::MatrixX3d ring(double r = 0.05, int n = 6)
{
  Eigen::MatrixX3d p(n, 3);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    p(i, 0) = r * std::cos(a);
    p(i, 1) = r * std::sin(a);
    p(i, 2) = 0.0;
  }
  return p;
}

class Fake : public rclcpp::Node
{
public:
  Fake()
  : Node("fake_detections")
  {
    k_intr_ = blindspot::intrinsics_from_fov(WIDTH, HEIGHT, FOVY);
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", latched);
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
    publish_info();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {tick();});
  }

private:
  void publish_info()
  {
    sensor_msgs::msg::CameraInfo m;
    m.width = WIDTH;
    m.height = HEIGHT;
    m.k = {k_intr_.fx, 0.0, k_intr_.cx, 0.0, k_intr_.fy, k_intr_.cy, 0.0, 0.0, 1.0};
    m.header.frame_id = "camera";
    info_pub_->publish(m);
  }

  void tick()
  {
    const int phase = (k_ / PHASE) % 3;
    Eigen::MatrixX3d pts = ring();
    std::vector<int> keep{0, 1, 2, 3, 4, 5};
    if (phase == 1) {
      keep = {0, 3};
    } else if (phase == 2) {
      pts.col(1) *= 0.02;
    }
    Eigen::Vector3d t(0.02 * std::sin(k_ * 0.05), -0.01, 0.62);

    Eigen::MatrixX2d uv(pts.rows(), 2);
    for (Eigen::Index i = 0; i < pts.rows(); ++i) {
      const Eigen::Vector3d p = pts.row(i).transpose() + t;
      uv(i, 0) = (p(0) / p(2)) * k_intr_.fx + k_intr_.cx;
      uv(i, 1) = (p(1) / p(2)) * k_intr_.fy + k_intr_.cy;
    }

    vision_msgs::msg::Detection2DArray msg;
    msg.header.frame_id = "camera";
    msg.header.stamp = now();
    // deliberately shuffled: a detector reports whatever order it finds, and
    // the guard node must recover the order from the ids.
    std::vector<int> order = keep;
    std::mt19937 rng(static_cast<unsigned int>(k_));
    std::shuffle(order.begin(), order.end(), rng);
    for (const int i : order) {
      vision_msgs::msg::Detection2D d;
      d.id = std::to_string(i);
      d.bbox.center.position.x = uv(i, 0);
      d.bbox.center.position.y = uv(i, 1);
      d.bbox.size_x = 40.0;
      d.bbox.size_y = 40.0;
      vision_msgs::msg::ObjectHypothesisWithPose h;
      h.hypothesis.class_id = std::to_string(i);
      h.hypothesis.score = 1.0;
      d.results.push_back(h);
      msg.detections.push_back(d);
    }
    det_pub_->publish(msg);

    if (k_ % PHASE == 0) {
      static const char * kNames[3] = {
        "healthy, 6 markers", "occluded, 2 markers", "collapsed target, 6 markers"};
      RCLCPP_INFO(get_logger(), "phase %d: %s", phase, kNames[phase]);
    }
    ++k_;
  }

  blindspot::Intrinsics k_intr_;
  int k_{0};
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fake>());
  rclcpp::shutdown();
  return 0;
}
