// Two UR5e cells, one folding part: the case the 2001 partition cannot survive.
//
// LEFT  cell: the 2001 partition stays on, always.
// RIGHT cell: the feature guard decides, per step.
//
// Mid-run both panels fold their two flaps away from the arm, so the six
// markers collapse toward the hinge line IN 3D while all six stay in view.
// The partition reads the shrinking polygon as "too far away" and drives the
// camera at the part. Both cells run the same protective stop.
//
// Measured offline first, with this exact control law - arm_checks includes
// the same header the arms run: at every fold tested the unguarded law lunges
// to 2-8 cm and loses the target, plain IBVS holds, sigma_6 never fires, and
// the area guard fires only when the fold outpaces the partition's own depth
// loop (85 deg in <= 2 s). Slower folds are regulated away by the lunge
// itself. This run uses 85 deg in 1 s.
//
// What this act does NOT show, measured in the first Gazebo run and
// documented in the README: held folded for several seconds, the guarded
// cell's fallback creeps (a fully folded part leaves an orbit about the hinge
// line that the image cannot see), and switching the partition back on during
// the unfold produced a 6.05 command spike that lost the target. The act ends
// 2.5 s after the fold completes for that reason (t_fold=20.0, a 1 s ramp,
// t_end=23.5), and the clip claims only what the act shows.
//
// Everything runs on SIM time, so recording at a reduced real-time factor
// changes nothing but wall-clock duration.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <opencv2/core.hpp>

#include "blindspot/arm_law.hpp"
#include "blindspot/calibrate.hpp"
#include "blindspot/detect.hpp"
#include "blindspot/guard.hpp"
#include "blindspot/kin.hpp"

namespace
{

using blindspot::arm_law::kDt;
using blindspot::arm_law::kRTarget;
using blindspot::arm_law::kZDesired;

/// Marker layout rotation. At 0 deg two marker pairs share a column and merge
/// in the image as the part folds; 11 deg keeps every column distinct.
const double kPhi = 11.0 * M_PI / 180.0;

/// Hinge centre, in each robot's frame.
const Eigen::Vector3d kPanel(0.636, -0.250, 0.359);

// act timeline, seconds of sim time (t_fold and t_end are ROS parameters)
constexpr double kTHome = 8.0;
constexpr double kFoldSecs = 1.0;
const double kFoldMax = 85.0 * M_PI / 180.0;

/// Protective stop, identical for both cells: no commanded motion may bring
/// the camera closer than this to the hinge plane. Checked on the NEXT pose.
constexpr double kStopStandoff = 0.12;

Eigen::MatrixX3d hexagon(double r)
{
  return blindspot::arm_law::hexagon(r, kPhi);
}

/// 0 until t_fold, then a 1 s ramp to kFoldMax, then held.
double fold_angle(double t, double t_fold)
{
  return kFoldMax * std::clamp((t - t_fold) / kFoldSecs, 0.0, 1.0);
}

/// Moore-Penrose pseudo-inverse with numpy.linalg.pinv's default cutoff.
///
/// numpy 2.x drops singular values at or below max(M, N) * eps * s_max. Eigen's
/// completeOrthogonalDecomposition().pseudoInverse() uses a different rank
/// test, so it is not substituted here: near a singular arm pose the two pick
/// different ranks and the joint command diverges.
Eigen::MatrixXd pinv_numpy(const Eigen::Ref<const Eigen::MatrixXd> & A)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd & S = svd.singularValues();
  const double rtol = static_cast<double>(std::max(A.rows(), A.cols())) *
    std::numeric_limits<double>::epsilon();
  const double cutoff = rtol * (S.size() > 0 ? S(0) : 0.0);
  Eigen::VectorXd Si(S.size());
  for (Eigen::Index i = 0; i < S.size(); ++i) {
    Si(i) = S(i) > cutoff ? 1.0 / S(i) : 0.0;
  }
  return svd.matrixV() * Si.asDiagonal() * svd.matrixU().transpose();
}

/// Runs xacro and returns the expanded URDF. Throws if it produces nothing.
std::string run_xacro(const std::string & path, const std::string & side)
{
  const std::string cmd = "xacro '" + path + "' ur_type:=ur5e prefix:=" + side +
    "_ ns:=/" + side;
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    throw std::runtime_error("could not run xacro");
  }
  char buf[4096];
  while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
    out += buf;
  }
  pclose(pipe);
  if (out.empty()) {
    throw std::runtime_error("xacro produced no URDF for side " + side);
  }
  return out;
}

class Cell
{
public:
  Cell(
    rclcpp::Node * node, const std::string & side,
    std::shared_ptr<blindspot::FeatureGuard> guard, bool guarded,
    const std::string & xacro_path)
  : side_(side), guard_(std::move(guard)), guarded_(guarded),
    chain_(run_xacro(xacro_path, side), "world", side + "_camera_optical_frame")
  {
    names_ = chain_.joint_names();
    pub_ = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/" + side + "/arm_controller/joint_trajectory", 10);
    fold_ = node->create_publisher<std_msgs::msg::Float64>("/fold_" + side, 10);
    img_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      "/" + side + "/camera/image", 10,
      [this](sensor_msgs::msg::Image::SharedPtr m) {
        // rgb8, row-major, as sensor_msgs lays it out.
        img_ = cv::Mat(
          static_cast<int>(m->height), static_cast<int>(m->width), CV_8UC3,
          m->data.data()).clone();
        have_img_ = true;
      });
    js_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/" + side + "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        std::map<std::string, double> d;
        for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
          d[m->name[i]] = m->position[i];
        }
        Eigen::VectorXd q(static_cast<Eigen::Index>(names_.size()));
        for (size_t i = 0; i < names_.size(); ++i) {
          const auto it = d.find(names_[i]);
          if (it == d.end()) {
            return;                         // not every joint reported yet
          }
          q(static_cast<Eigen::Index>(i)) = it->second;
        }
        q_ = q;
        have_q_ = true;
      });
  }

  const std::string & side() const {return side_;}
  bool have_q() const {return have_q_;}
  const Eigen::VectorXd & q() const {return q_;}
  const blindspot::Chain & chain() const {return chain_;}
  int n_feat() const {return n_feat_;}
  double err() const {return err_;}
  bool lost() const {return lost_;}
  bool used_partition() const {return used_partition_;}
  const Eigen::Matrix<double, 6, 1> & v() const {return v_;}
  bool has_reading() const {return has_reading_;}
  const blindspot::GuardReading & reading() const {return reading_;}
  bool stopped() const {return stopped_;}
  double stopped_at() const {return stopped_at_;}

  /// Camera optical centre to the hinge plane, along the panel normal.
  double standoff(const Eigen::Ref<const Eigen::VectorXd> & q) const
  {
    return chain_.fk(q)(1, 3) - kPanel(1);
  }

  void publish_fold(double a)
  {
    std_msgs::msg::Float64 m;
    m.data = a;
    fold_->publish(m);
  }

  void send(const Eigen::Ref<const Eigen::VectorXd> & q, double secs)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = names_;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(q.data(), q.data() + q.size());
    pt.time_from_start.sec = static_cast<int32_t>(secs);
    pt.time_from_start.nanosec =
      static_cast<uint32_t>((secs - static_cast<int32_t>(secs)) * 1e9);
    msg.points.push_back(pt);
    pub_->publish(msg);
  }

  void go_home()
  {
    Eigen::VectorXd home(6);
    home << 0.0, -1.2, 1.4, -1.75, -1.57, 0.0;
    send(home, 3.0);
  }

  void step(double t)
  {
    v_.setZero();
    if (!have_img_ || !have_q_ || stopped_) {
      return;
    }
    const auto k = blindspot::arm_law::default_intrinsics();
    const Eigen::MatrixX2d uv_raw = blindspot::detect_dots(img_);
    n_feat_ = static_cast<int>(uv_raw.rows());
    if (uv_raw.rows() < 3) {
      lost_ = true;
      return;
    }
    const Eigen::MatrixX2d uv = blindspot::arm_law::order_by_angle(uv_raw);
    Eigen::MatrixX2d s(uv.rows(), 2);
    s.col(0) = (uv.col(0).array() - k.cx) / k.fx;
    s.col(1) = (uv.col(1).array() - k.cy) / k.fy;

    reading_ = guard_->evaluate(s);
    has_reading_ = true;
    if (s.rows() != 6) {
      lost_ = true;
      return;
    }
    lost_ = false;

    const Eigen::MatrixX3d hex = hexagon(kRTarget / kZDesired);
    const Eigen::MatrixX2d s_star = blindspot::arm_law::best_cyclic_match(
      s, blindspot::arm_law::order_by_angle(hex.leftCols<2>()));

    used_partition_ = guarded_ ? reading_.decision : true;
    const Eigen::Matrix<double, 6, 1> v_cam =
      blindspot::arm_law::ibvs_twist(s, s_star, used_partition_);
    err_ = (s - s_star).norm();

    const Eigen::Matrix4d T = chain_.fk(q_);
    const Eigen::Matrix3d Rbc = T.topLeftCorner<3, 3>();
    Eigen::Matrix<double, 6, 1> tw;
    tw.head<3>() = Rbc * v_cam.head<3>();
    tw.tail<3>() = Rbc * v_cam.tail<3>();

    const Eigen::MatrixXd J = chain_.jacobian(q_);
    Eigen::VectorXd qd = pinv_numpy(J) * tw;
    qd = qd.cwiseMax(-1.5).cwiseMin(1.5);
    const Eigen::VectorXd q_next = q_ + qd * kDt;

    if (standoff(q_next) < kStopStandoff) {
      // protective stop: hold where we are and latch, as a real cell would
      // until an operator resets it
      stopped_ = true;
      stopped_at_ = t;
      send(q_, 0.16);
      return;
    }
    v_ = v_cam;
    send(q_next, kDt * 1.6);
  }

private:
  std::string side_;
  std::shared_ptr<blindspot::FeatureGuard> guard_;
  bool guarded_{false};
  blindspot::Chain chain_;
  std::vector<std::string> names_;

  cv::Mat img_;
  bool have_img_{false};
  Eigen::VectorXd q_;
  bool have_q_{false};

  blindspot::GuardReading reading_{};
  bool has_reading_{false};
  Eigen::Matrix<double, 6, 1> v_{Eigen::Matrix<double, 6, 1>::Zero()};
  double err_{0.0};
  int n_feat_{0};
  bool used_partition_{true};
  bool stopped_{false};
  double stopped_at_{0.0};
  bool lost_{false};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fold_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
};

class Fold : public rclcpp::Node
{
public:
  Fold()
  : Node("blindspot_fold", rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true)}))
  {
    declare_parameter<std::string>("log", "/tmp/blindspot_fold_log.csv");
    declare_parameter<double>("t_fold", 20.0);
    declare_parameter<double>("t_end", 23.5);
    t_fold_ = get_parameter("t_fold").as_double();
    t_end_ = get_parameter("t_end").as_double();

    Eigen::Matrix4d goal = Eigen::Matrix4d::Identity();
    goal(2, 3) = kZDesired;
    blindspot::CalibrateOptions opts;
    opts.n_poses = 600;
    opts.seed = 0;
    opts.z_range = std::make_pair(0.18, 0.40);
    opts.lateral = 0.05;
    const auto cal = blindspot::calibrate(hexagon(kRTarget), goal, opts);
    auto guard = std::make_shared<blindspot::FeatureGuard>(cal);

    std::string table;
    for (const auto & [n, v] : cal.thresholds()) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%d: %.3e", n, v);
      table += (table.empty() ? "" : ", ") + std::string(buf);
    }
    RCLCPP_INFO(get_logger(), "thresholds: {%s}", table.c_str());

    const std::string xacro_path =
      (ament_index_cpp::get_package_share_path("blindspot_arm") /
      "urdf" / "ur5e_camera.urdf.xacro").string();
    left_ = std::make_unique<Cell>(this, "left", guard, false, xacro_path);
    right_ = std::make_unique<Cell>(this, "right", guard, true, xacro_path);

    const auto path = get_parameter("log").as_string();
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    log_.open(path);
    log_ << "sim,t,fold,side,n_feat,err,signal,threshold,margin,partition,"
      "v_norm,v_lin,vz,standoff,stopped_at,lost,cam_x,cam_y,cam_z,"
      "q0,q1,q2,q3,q4,q5\n";

    timer_ = create_wall_timer(
      std::chrono::duration<double>(kDt), [this]() {tick();});
  }

private:
  void tick()
  {
    const double nowsec = get_clock()->now().nanoseconds() * 1e-9;
    if (nowsec <= 0.0) {
      return;                               // /clock not yet received
    }
    if (!have_t0_) {
      t0_ = nowsec;
      have_t0_ = true;
    }
    const double t = nowsec - t0_;
    const double a = fold_angle(t, t_fold_);

    for (Cell * c : {left_.get(), right_.get()}) {
      c->publish_fold(a);
      if (t < kTHome) {
        c->go_home();
      } else {
        c->step(t);
      }
      if (c->have_q()) {
        write_row(nowsec, t, a, *c);
      }
    }
    log_.flush();

    if (static_cast<int>(t * 10) % 10 == 0 && t > kTHome) {
      RCLCPP_INFO(
        get_logger(),
        "t=%5.1f fold=%4.2f | L n=%d m=%.2f d=%.3f stop=%s | "
        "R n=%d m=%.2f d=%.3f part=%d stop=%s",
        t, a, left_->n_feat(),
        left_->has_reading() ? left_->reading().margin : 0.0,
        left_->have_q() ? left_->standoff(left_->q()) : 0.0,
        left_->stopped() ? "yes" : "none", right_->n_feat(),
        right_->has_reading() ? right_->reading().margin : 0.0,
        right_->have_q() ? right_->standoff(right_->q()) : 0.0,
        right_->used_partition() ? 1 : 0, right_->stopped() ? "yes" : "none");
    }

    if (t > t_end_ && !done_) {
      done_ = true;
      log_.close();
      RCLCPP_INFO(get_logger(), "act complete");
      rclcpp::shutdown();
    }
  }

  void write_row(double nowsec, double t, double a, const Cell & c)
  {
    const auto & q = c.q();
    const Eigen::Vector3d cam = c.chain().fk(q).topRightCorner<3, 1>();
    const double sig = c.has_reading() ? c.reading().signal : 0.0;
    const double thr = c.has_reading() ? c.reading().threshold : 0.0;
    const double mar = c.has_reading() ? c.reading().margin : 0.0;
    char buf[640];
    std::snprintf(
      buf, sizeof(buf),
      "%.3f,%.2f,%.4f,%s,%d,%.5f,%.5e,%.5e,%.4f,%d,%.4f,%.4f,%.4f,%.4f,",
      nowsec, t, a, c.side().c_str(), c.n_feat(), c.err(), sig, thr, mar,
      c.used_partition() ? 1 : 0, c.v().norm(), c.v().head<3>().norm(), c.v()(2),
      c.standoff(q));
    log_ << buf;
    if (c.stopped()) {
      std::snprintf(buf, sizeof(buf), "%.2f", c.stopped_at());
      log_ << buf;
    }
    std::snprintf(
      buf, sizeof(buf), ",%d,%.4f,%.4f,%.4f", c.lost() ? 1 : 0, cam(0), cam(1), cam(2));
    log_ << buf;
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      std::snprintf(buf, sizeof(buf), ",%.5f", q(i));
      log_ << buf;
    }
    log_ << "\n";
  }

  double t_fold_{20.0};
  double t_end_{23.5};
  double t0_{0.0};
  bool have_t0_{false};
  bool done_{false};
  std::unique_ptr<Cell> left_;
  std::unique_ptr<Cell> right_;
  std::ofstream log_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Fold>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("blindspot_fold"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
