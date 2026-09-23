// Live IBVS simulation, closed through ROS around the guard node.
//
// This is the whole point made visible: a servo loop whose control law is
// chosen, every step, by the Bool the guard node publishes.
//
//     partition_ok == true   -> partitioned law (2001), good on retreat
//     partition_ok == false  -> plain truncated law, safe when features go bad
//
// It cycles through three regimes so the guard visibly changes its mind:
// healthy, occluded down to 2 markers, and a target collapsed toward a line.
//
// Publishes
//     detections      vision_msgs/Detection2DArray   shuffled, with ids
//     camera_info     sensor_msgs/CameraInfo
//     camera/image    sensor_msgs/Image              the view, with an overlay
//     markers         visualization_msgs/MarkerArray target and camera, in 3D
//     tf              world -> camera
//
// Everything is drawn by hand into the Image buffer, so there is no cv_bridge
// or OpenCV dependency in this node.
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "blindspot/units.hpp"

namespace
{

constexpr int W = 640;
constexpr int H = 480;
constexpr double FOVY = 45.0;
constexpr double DT = 0.1;
constexpr int PHASE = 60;
constexpr double TAU = 1e-3;
constexpr double LAM = 0.5;

const std::array<int, 2> Z_COLS{2, 5};        // vz, wz - from image measurements
const std::array<int, 4> XY_COLS{0, 1, 3, 4};  // vx, vy, wx, wy - reduced inverse

using Colour = std::array<uint8_t, 3>;

// ---------------------------------------------------------------------------
// maths
// ---------------------------------------------------------------------------

double polygon_area_sqrt(const Eigen::Ref<const Eigen::MatrixX2d> & s)
{
  double cross = 0.0;
  for (Eigen::Index i = 0; i < s.rows(); ++i) {
    const Eigen::Index j = (i + 1) % s.rows();
    cross += s(i, 0) * s(j, 1) - s(i, 1) * s(j, 0);
  }
  return std::sqrt(std::max(0.5 * std::abs(cross), 1e-12));
}

double line_angle(const Eigen::Ref<const Eigen::MatrixX2d> & s)
{
  return std::atan2(s(1, 1) - s(0, 1), s(1, 0) - s(0, 0));
}

double wrap(double a)
{
  const double two_pi = 2.0 * M_PI;
  double m = std::fmod(a + M_PI, two_pi);
  if (m < 0.0) {
    m += two_pi;
  }
  return m - M_PI;
}

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

Eigen::Matrix3d skew(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d K;
  K << 0.0, -w(2), w(1),
    w(2), 0.0, -w(0),
    -w(1), w(0), 0.0;
  return K;
}

Eigen::Matrix4d se3_exp(const Eigen::Matrix<double, 6, 1> & xi)
{
  const Eigen::Vector3d v = xi.head<3>();
  const Eigen::Vector3d w = xi.tail<3>();
  const double th = w.norm();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (th < 1e-12) {
    T.topRightCorner<3, 1>() = v;
    return T;
  }
  const Eigen::Matrix3d Wm = skew(w / th);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  T.topLeftCorner<3, 3>() = I + std::sin(th) * Wm + (1.0 - std::cos(th)) * (Wm * Wm);
  const Eigen::Matrix3d V =
    I + ((1.0 - std::cos(th)) / th) * Wm + ((th - std::sin(th)) / th) * (Wm * Wm);
  T.topRightCorner<3, 1>() = V * v;
  return T;
}

Eigen::MatrixXd interaction_matrix(
  const Eigen::Ref<const Eigen::MatrixX2d> & s, const Eigen::Ref<const Eigen::VectorXd> & Z)
{
  const Eigen::Index n = s.rows();
  Eigen::MatrixXd L(2 * n, 6);
  for (Eigen::Index i = 0; i < n; ++i) {
    const double x = s(i, 0);
    const double y = s(i, 1);
    const double Zi = Z(i);
    L.row(2 * i) << -1.0 / Zi, 0.0, x / Zi, x * y, -(1.0 + x * x), y;
    L.row(2 * i + 1) << 0.0, -1.0 / Zi, y / Zi, 1.0 + y * y, -x * y, -x;
  }
  return L;
}

/// Pseudo-inverse with singular directions below rel_tau*s_max dropped.
/// Unlike arm_law::pinv_trunc this keeps the first direction when the whole
/// spectrum falls below the tolerance - it is the study's reference
/// controller, and that fallback is part of it.
Eigen::MatrixXd pinv_truncated(
  const Eigen::Ref<const Eigen::MatrixXd> & L, double rel_tau, int * rank = nullptr)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(L, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd & S = svd.singularValues();
  std::vector<bool> keep(static_cast<size_t>(S.size()));
  int kept = 0;
  for (Eigen::Index i = 0; i < S.size(); ++i) {
    keep[static_cast<size_t>(i)] = S(i) > rel_tau * S(0);
    kept += keep[static_cast<size_t>(i)] ? 1 : 0;
  }
  if (kept == 0) {
    keep[0] = true;
    kept = 1;
  }
  Eigen::VectorXd Si(S.size());
  for (Eigen::Index i = 0; i < S.size(); ++i) {
    Si(i) = keep[static_cast<size_t>(i)] ? (S(i) > 0.0 ? 1.0 / S(i) : 1.0) : 0.0;
  }
  if (rank != nullptr) {
    *rank = kept;
  }
  return svd.matrixV() * Si.asDiagonal() * svd.matrixU().transpose();
}

// ---------------------------------------------------------------------------
// a 5x7 bitmap font, enough for a heads-up overlay without pulling in OpenCV
// ---------------------------------------------------------------------------

const std::map<char, std::array<const char *, 7>> & glyphs()
{
  static const std::map<char, std::array<const char *, 7>> g{
    {'0', {"01110", "10001", "10011", "10101", "11001", "10001", "01110"}},
    {'1', {"00100", "01100", "00100", "00100", "00100", "00100", "01110"}},
    {'2', {"01110", "10001", "00001", "00010", "00100", "01000", "11111"}},
    {'3', {"11111", "00010", "00100", "00010", "00001", "10001", "01110"}},
    {'4', {"00010", "00110", "01010", "10010", "11111", "00010", "00010"}},
    {'5', {"11111", "10000", "11110", "00001", "00001", "10001", "01110"}},
    {'6', {"00110", "01000", "10000", "11110", "10001", "10001", "01110"}},
    {'7', {"11111", "00001", "00010", "00100", "01000", "01000", "01000"}},
    {'8', {"01110", "10001", "10001", "01110", "10001", "10001", "01110"}},
    {'9', {"01110", "10001", "10001", "01111", "00001", "00010", "01100"}},
    {'.', {"00000", "00000", "00000", "00000", "00000", "01100", "01100"}},
    {'X', {"00000", "00000", "10001", "01010", "00100", "01010", "10001"}},
    {'/', {"00001", "00010", "00010", "00100", "01000", "01000", "10000"}},
    {'-', {"00000", "00000", "00000", "11111", "00000", "00000", "00000"}},
    {' ', {"00000", "00000", "00000", "00000", "00000", "00000", "00000"}},
    {'A', {"01110", "10001", "10001", "11111", "10001", "10001", "10001"}},
    {'B', {"11110", "10001", "10001", "11110", "10001", "10001", "11110"}},
    {'C', {"01110", "10001", "10000", "10000", "10000", "10001", "01110"}},
    {'D', {"11110", "10001", "10001", "10001", "10001", "10001", "11110"}},
    {'E', {"11111", "10000", "10000", "11110", "10000", "10000", "11111"}},
    {'F', {"11111", "10000", "10000", "11110", "10000", "10000", "10000"}},
    {'G', {"01110", "10001", "10000", "10111", "10001", "10001", "01111"}},
    {'H', {"10001", "10001", "10001", "11111", "10001", "10001", "10001"}},
    {'I', {"11111", "00100", "00100", "00100", "00100", "00100", "11111"}},
    {'J', {"00111", "00010", "00010", "00010", "00010", "10010", "01100"}},
    {'K', {"10001", "10010", "10100", "11000", "10100", "10010", "10001"}},
    {'L', {"10000", "10000", "10000", "10000", "10000", "10000", "11111"}},
    {'M', {"10001", "11011", "10101", "10101", "10001", "10001", "10001"}},
    {'N', {"10001", "11001", "10101", "10011", "10001", "10001", "10001"}},
    {'O', {"01110", "10001", "10001", "10001", "10001", "10001", "01110"}},
    {'P', {"11110", "10001", "10001", "11110", "10000", "10000", "10000"}},
    {'Q', {"01110", "10001", "10001", "10001", "10101", "10010", "01101"}},
    {'R', {"11110", "10001", "10001", "11110", "10100", "10010", "10001"}},
    {'S', {"01111", "10000", "10000", "01110", "00001", "00001", "11110"}},
    {'T', {"11111", "00100", "00100", "00100", "00100", "00100", "00100"}},
    {'U', {"10001", "10001", "10001", "10001", "10001", "10001", "01110"}},
    {'V', {"10001", "10001", "10001", "10001", "10001", "01010", "00100"}},
    {'W', {"10001", "10001", "10001", "10101", "10101", "11011", "10001"}},
    {'Y', {"10001", "10001", "01010", "00100", "00100", "00100", "00100"}},
    {'Z', {"11111", "00001", "00010", "00100", "01000", "10000", "11111"}},
  };
  return g;
}

/// Row-major RGB8 image buffer, laid out exactly as sensor_msgs/Image wants.
class Canvas
{
public:
  Canvas(int w, int h, uint8_t fill)
  : w_(w), h_(h), data_(static_cast<size_t>(w) * h * 3, fill) {}

  void set(int x, int y, const Colour & c)
  {
    if (x < 0 || y < 0 || x >= w_ || y >= h_) {
      return;
    }
    const size_t i = (static_cast<size_t>(y) * w_ + x) * 3;
    data_[i] = c[0];
    data_[i + 1] = c[1];
    data_[i + 2] = c[2];
  }

  void fill_rows(int y0, int y1, const Colour & c)
  {
    for (int y = std::max(0, y0); y < std::min(h_, y1); ++y) {
      for (int x = 0; x < w_; ++x) {
        set(x, y, c);
      }
    }
  }

  void fill_col(int x, const Colour & c)
  {
    for (int y = 0; y < h_; ++y) {
      set(x, y, c);
    }
  }

  void fill_row(int y, const Colour & c)
  {
    for (int x = 0; x < w_; ++x) {
      set(x, y, c);
    }
  }

  void disc(double cx, double cy, double rad, const Colour & c)
  {
    const int y0 = static_cast<int>(std::floor(cy - rad));
    const int y1 = static_cast<int>(std::floor(cy + rad)) + 1;
    const int x0 = static_cast<int>(std::floor(cx - rad));
    const int x1 = static_cast<int>(std::floor(cx + rad)) + 1;
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        const double dy = y - cy;
        const double dx = x - cx;
        if (dx * dx + dy * dy <= rad * rad) {
          set(x, y, c);
        }
      }
    }
  }

  void line(
    double px, double py, double qx, double qy, const Colour & c, double width = 1.0)
  {
    const int n = static_cast<int>(std::max(std::abs(qx - px), std::abs(qy - py))) + 1;
    for (int i = 0; i < n; ++i) {
      const double t = n == 1 ? 0.0 : static_cast<double>(i) / (n - 1);
      disc(px + (qx - px) * t, py + (qy - py) * t, width, c);
    }
  }

  int text(int x, int y, const std::string & s, const Colour & c, int scale = 2)
  {
    for (char ch : s) {
      const char up = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
      const auto it = glyphs().find(up);
      const auto & g = it != glyphs().end() ? it->second : glyphs().at(' ');
      for (int r = 0; r < 7; ++r) {
        const char * row = g[static_cast<size_t>(r)];
        for (int col = 0; col < 5; ++col) {
          if (row[col] == '1') {
            for (int dy = 0; dy < scale; ++dy) {
              for (int dx = 0; dx < scale; ++dx) {
                set(x + col * scale + dx, y + r * scale + dy, c);
              }
            }
          }
        }
      }
      x += 6 * scale;
    }
    return x;
  }

  const std::vector<uint8_t> & data() const {return data_;}

private:
  int w_;
  int h_;
  std::vector<uint8_t> data_;
};

class Sim : public rclcpp::Node
{
public:
  Sim()
  : Node("blindspot_sim")
  {
    k_intr_ = blindspot::intrinsics_from_fov(W, H, FOVY);
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", latched);
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
    img_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    mk_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    guard_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/blindspot_guard/partition_ok", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {partition_ok_ = m->data;});

    reset();
    publish_info();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(DT), [this]() {tick();});
  }

private:
  /// Fresh offset pose, so every phase shows a real approach.
  ///
  /// Without this the servo converges in a couple of seconds and then sits
  /// perfectly still, which looks identical to nothing happening.
  void reset()
  {
    Eigen::Matrix<double, 6, 1> xi;
    xi << 0.0, 0.0, 0.0, 0.25, 0.15, 0.4;
    cTo_ = Eigen::Matrix4d::Identity();
    cTo_.topLeftCorner<3, 3>() = se3_exp(xi).topLeftCorner<3, 3>();
    cTo_.topRightCorner<3, 1>() = Eigen::Vector3d(0.06, -0.05, 0.88);
  }

  struct Target
  {
    Eigen::MatrixX3d pts;
    std::vector<int> keep;
    int phase{0};
  };

  Target target() const
  {
    const int phase = (k_ / PHASE) % 3;
    Eigen::MatrixX3d pts = ring();
    if (phase == 2) {
      pts.col(1) *= 0.02;                   // collapsed toward a line
    }
    std::vector<int> keep;
    if (phase == 1) {
      keep = {0, 3};                        // occluded down to 2 markers
    } else {
      keep = {0, 1, 2, 3, 4, 5};
    }
    return Target{pts, keep, phase};
  }

  void publish_info()
  {
    sensor_msgs::msg::CameraInfo m;
    m.width = W;
    m.height = H;
    m.k = {k_intr_.fx, 0.0, k_intr_.cx, 0.0, k_intr_.fy, k_intr_.cy, 0.0, 0.0, 1.0};
    m.header.frame_id = "camera";
    info_pub_->publish(m);
  }

  static Eigen::MatrixX2d project(const Eigen::MatrixX3d & P_c)
  {
    Eigen::MatrixX2d s(P_c.rows(), 2);
    s.col(0) = P_c.col(0).array() / P_c.col(2).array();
    s.col(1) = P_c.col(1).array() / P_c.col(2).array();
    return s;
  }

  static Eigen::MatrixX3d transform(const Eigen::Matrix4d & T, const Eigen::MatrixX3d & P)
  {
    Eigen::MatrixX3d out(P.rows(), 3);
    for (Eigen::Index i = 0; i < P.rows(); ++i) {
      out.row(i) =
        (T.topLeftCorner<3, 3>() * P.row(i).transpose() + T.topRightCorner<3, 1>())
        .transpose();
    }
    return out;
  }

  static Eigen::MatrixX2d rows_of(const Eigen::MatrixX2d & m, const std::vector<int> & idx)
  {
    Eigen::MatrixX2d out(static_cast<Eigen::Index>(idx.size()), 2);
    for (size_t i = 0; i < idx.size(); ++i) {
      out.row(static_cast<Eigen::Index>(i)) = m.row(idx[i]);
    }
    return out;
  }

  void tick()
  {
    const Target tg = target();
    if (tg.phase != phase_) {               // new regime, new approach to watch
      phase_ = tg.phase;
      reset();
    }
    Eigen::MatrixX3d P_c = transform(cTo_, tg.pts);
    if ((P_c.col(2).array() < 1e-3).any()) {
      reset();
      return;
    }
    const Eigen::MatrixX2d s = project(P_c);

    Eigen::Matrix4d goal = Eigen::Matrix4d::Identity();
    goal(2, 3) = 0.6;
    const Eigen::MatrixX2d s_star = project(transform(goal, tg.pts));

    const Eigen::MatrixX2d s_k = rows_of(s, tg.keep);
    const Eigen::MatrixX2d s_star_k = rows_of(s_star, tg.keep);
    Eigen::VectorXd e(2 * static_cast<Eigen::Index>(tg.keep.size()));
    for (size_t i = 0; i < tg.keep.size(); ++i) {
      e(2 * static_cast<Eigen::Index>(i)) = s_k(static_cast<Eigen::Index>(i), 0) -
        s_star_k(static_cast<Eigen::Index>(i), 0);
      e(2 * static_cast<Eigen::Index>(i) + 1) = s_k(static_cast<Eigen::Index>(i), 1) -
        s_star_k(static_cast<Eigen::Index>(i), 1);
    }
    Eigen::VectorXd Zk(static_cast<Eigen::Index>(tg.keep.size()));
    for (size_t i = 0; i < tg.keep.size(); ++i) {
      Zk(static_cast<Eigen::Index>(i)) = P_c(tg.keep[i], 2);
    }
    const Eigen::MatrixXd L = interaction_matrix(s_k, Zk);

    Eigen::Matrix<double, 6, 1> v = Eigen::Matrix<double, 6, 1>::Zero();
    if (partition_ok_ && tg.keep.size() >= 3) {
      // the 2001 partition: vz from the polygon area, wz from the line angle
      // between two designated features, the other four DOF from the reduced
      // interaction matrix with the coupling term ADDED.
      const double sig = polygon_area_sqrt(s_k);
      const double sig_star = polygon_area_sqrt(s_star_k);
      const double vz = 0.6 * std::log(sig_star / std::max(sig, 1e-9));
      const double wz = 0.6 * wrap(line_angle(s_k) - line_angle(s_star_k));
      Eigen::MatrixXd L_xy(L.rows(), 4);
      for (int c = 0; c < 4; ++c) {
        L_xy.col(c) = L.col(XY_COLS[static_cast<size_t>(c)]);
      }
      Eigen::MatrixXd L_z(L.rows(), 2);
      for (int c = 0; c < 2; ++c) {
        L_z.col(c) = L.col(Z_COLS[static_cast<size_t>(c)]);
      }
      const Eigen::Vector2d vz_wz(vz, wz);
      const Eigen::VectorXd v_xy = -pinv_truncated(L_xy, TAU) * (LAM * e + L_z * vz_wz);
      for (int c = 0; c < 4; ++c) {
        v(XY_COLS[static_cast<size_t>(c)]) = v_xy(c);
      }
      v(2) = vz;
      v(5) = wz;
    } else {
      // the guard dropped the partition: plain truncated law on full L
      v = -LAM * (pinv_truncated(L, TAU) * e);
    }
    v = v.cwiseMax(-3.0).cwiseMin(3.0);
    cTo_ = se3_exp(-v * DT) * cTo_;

    publish_detections(s, tg.keep);
    publish_image(s, tg.keep, tg.phase, v.norm());
    publish_markers(tg.pts, tg.keep);
    publish_tf();
    ++k_;
  }

  Eigen::MatrixX2d to_pixels(const Eigen::MatrixX2d & s) const
  {
    Eigen::MatrixX2d uv(s.rows(), 2);
    uv.col(0) = s.col(0).array() * k_intr_.fx + k_intr_.cx;
    uv.col(1) = s.col(1).array() * k_intr_.fy + k_intr_.cy;
    return uv;
  }

  void publish_detections(const Eigen::MatrixX2d & s, const std::vector<int> & keep)
  {
    const Eigen::MatrixX2d uv = to_pixels(s);
    vision_msgs::msg::Detection2DArray msg;
    msg.header.frame_id = "camera";
    msg.header.stamp = now();
    // Shuffled on purpose: the guard must put them back in order itself.
    std::vector<int> order = keep;
    std::mt19937 rng(static_cast<unsigned int>(k_));
    std::shuffle(order.begin(), order.end(), rng);
    for (const int i : order) {
      vision_msgs::msg::Detection2D d;
      d.id = std::to_string(i);
      d.bbox.center.position.x = uv(i, 0);
      d.bbox.center.position.y = uv(i, 1);
      d.bbox.size_x = 30.0;
      d.bbox.size_y = 30.0;
      vision_msgs::msg::ObjectHypothesisWithPose h;
      h.hypothesis.class_id = std::to_string(i);
      h.hypothesis.score = 1.0;
      d.results.push_back(h);
      msg.detections.push_back(d);
    }
    det_pub_->publish(msg);
  }

  void publish_image(
    const Eigen::MatrixX2d & s, const std::vector<int> & keep, int phase, double speed)
  {
    const Eigen::MatrixX2d uv = to_pixels(s);
    Canvas img(W, H, 26);
    for (int gx = 0; gx < W; gx += 40) {
      img.fill_col(gx, Colour{38, 38, 38});
    }
    for (int gy = 0; gy < H; gy += 40) {
      img.fill_row(gy, Colour{38, 38, 38});
    }

    const bool ok = partition_ok_;
    const Colour col = ok ? Colour{60, 200, 90} : Colour{235, 90, 60};
    if (keep.size() >= 3) {
      for (size_t i = 0; i < keep.size(); ++i) {
        const int a = keep[i];
        const int b = keep[(i + 1) % keep.size()];
        img.line(uv(a, 0), uv(a, 1), uv(b, 0), uv(b, 1), Colour{90, 90, 110}, 1.0);
      }
    }
    for (const int i : keep) {
      img.disc(uv(i, 0), uv(i, 1), 7, col);
      img.disc(uv(i, 0), uv(i, 1), 3, Colour{255, 255, 255});
    }

    img.fill_rows(0, 34, Colour{45, 45, 55});
    static const char * kPhase[3] = {
      "HEALTHY 6 MARKERS", "OCCLUDED 2 MARKERS", "COLLAPSED TARGET"};
    img.text(8, 10, kPhase[phase], Colour{230, 230, 240}, 2);
    img.fill_rows(H - 40, H, Colour{45, 45, 55});
    img.text(8, H - 32, ok ? "PARTITION OK" : "PARTITION DROPPED", col, 2);
    char sp[32];
    std::snprintf(sp, sizeof(sp), "V %.2f", speed);
    img.text(300, H - 32, sp, Colour{200, 200, 210}, 2);
    img.fill_rows(34, 40, col);

    sensor_msgs::msg::Image m;
    m.header.frame_id = "camera";
    m.header.stamp = now();
    m.height = H;
    m.width = W;
    m.encoding = "rgb8";
    m.step = W * 3;
    m.data = img.data();
    img_pub_->publish(m);
  }

  void publish_markers(const Eigen::MatrixX3d & pts, const std::vector<int> & keep)
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();
    for (Eigen::Index i = 0; i < pts.rows(); ++i) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = stamp;
      mk.ns = "target";
      mk.id = static_cast<int>(i);
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = pts(i, 0);
      mk.pose.position.y = pts(i, 1);
      mk.pose.position.z = pts(i, 2);
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.012;
      const bool seen =
        std::find(keep.begin(), keep.end(), static_cast<int>(i)) != keep.end();
      mk.color.r = seen ? 0.25 : 0.55;
      mk.color.g = seen ? 0.85 : 0.55;
      mk.color.b = seen ? 0.35 : 0.55;
      mk.color.a = seen ? 1.0 : 0.25;
      arr.markers.push_back(mk);
    }
    mk_pub_->publish(arr);
  }

  void publish_tf()
  {
    const Eigen::Matrix3d R = cTo_.topLeftCorner<3, 3>().transpose();
    const Eigen::Vector3d t = -R * cTo_.topRightCorner<3, 1>();
    // Eigen covers all four branches of the matrix-to-quaternion conversion.
    // Hand-rolling only the trace>0 branch and falling back to identity looks
    // fine until the camera passes 180 degrees, and then RViz quietly shows
    // the wrong orientation.
    const Eigen::Quaterniond q(R);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera";
    tf.transform.translation.x = t(0);
    tf.transform.translation.y = t(1);
    tf.transform.translation.z = t(2);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_->sendTransform(tf);
  }

  blindspot::Intrinsics k_intr_;
  Eigen::Matrix4d cTo_{Eigen::Matrix4d::Identity()};
  bool partition_ok_{true};
  int k_{0};
  int phase_{-1};

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mk_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr guard_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sim>());
  rclcpp::shutdown();
  return 0;
}
