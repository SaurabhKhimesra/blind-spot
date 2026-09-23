// Save every frame of the film and wrist cameras, named by SIM timestamp.
//
// The clip is assembled offline from these frames and the fold node's log, so
// every pixel of robot motion in it was rendered by Gazebo during the run.
//
//     ros2 run blindspot_cpp record_node --ros-args -p out:=/tmp/blindspot_frames
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace
{

const std::map<std::string, std::string> kTopics{
  {"cine_left", "/cine_left"},
  {"cine_right", "/cine_right"},
  {"wrist_left", "/left/camera/image"},
  {"wrist_right", "/right/camera/image"},
};

class Recorder : public rclcpp::Node
{
public:
  Recorder()
  : Node("blindspot_record", rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true)}))
  {
    declare_parameter<std::string>("out", "/tmp/blindspot_frames");
    out_ = get_parameter("out").as_string();

    // imwrite is slow enough to drop frames if it runs on the executor
    // thread, so each topic gets its own callback group and the executor is
    // multithreaded. A worker pool would do the same job with one more queue
    // to reason about.
    rclcpp::QoS qos(200);
    qos.reliable().keep_last(200);
    for (const auto & [name, topic] : kTopics) {
      std::filesystem::create_directories(std::filesystem::path(out_) / name);
      counts_[name] = 0;
      auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      groups_.push_back(group);
      auto opts = rclcpp::SubscriptionOptions();
      opts.callback_group = group;
      subs_.push_back(
        create_subscription<sensor_msgs::msg::Image>(
          topic, qos,
          [this, name](sensor_msgs::msg::Image::SharedPtr m) {on_img(name, *m);},
          opts));
    }
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {report();});
  }

private:
  void on_img(const std::string & name, const sensor_msgs::msg::Image & m)
  {
    const int64_t stamp =
      static_cast<int64_t>(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
    const cv::Mat rgb(
      static_cast<int>(m.height), static_cast<int>(m.width), CV_8UC3,
      const_cast<uint8_t *>(m.data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

    char fname[32];
    std::snprintf(fname, sizeof(fname), "%015ld.jpg", static_cast<long>(stamp));
    const auto path = std::filesystem::path(out_) / name / fname;
    cv::imwrite(path.string(), bgr, {cv::IMWRITE_JPEG_QUALITY, 94});
    ++counts_[name];
  }

  void report()
  {
    std::string s;
    for (const auto & [name, n] : counts_) {
      s += (s.empty() ? "" : ", ") + name + ": " + std::to_string(n.load());
    }
    RCLCPP_INFO(get_logger(), "frames: {%s}", s.c_str());
  }

  std::string out_;
  std::map<std::string, std::atomic<int>> counts_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> groups_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Recorder>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 6);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
