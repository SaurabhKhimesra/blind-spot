// The eye-in-hand control law and perception shared by the arm nodes.
//
// No ROS includes and no OpenCV: the Gazebo node and the checks include this
// same header, so the checks exercise the exact law the arms run. The marker
// detector lives in detect.hpp.
#ifndef BLINDSPOT__ARM_LAW_HPP_
#define BLINDSPOT__ARM_LAW_HPP_

#include <Eigen/Core>

#include "blindspot/units.hpp"

namespace blindspot
{
namespace arm_law
{

constexpr double kRTarget = 0.05;    ///< fiducial ring radius on the panel, m
constexpr double kZDesired = 0.26;   ///< standoff the servo drives to
constexpr double kLam = 0.6;
constexpr double kTau = 1e-3;
constexpr double kDt = 0.1;

// Intrinsics come from the sensor definition in the URDF, not from
// camera_info. The gz topic /left_wrist_camera_info exists but nothing
// arrives on the ROS side, and waiting on it silently stalled the whole servo
// loop: step() returned early, so the HUD read 0/6 features and neither arm
// moved. Principal point is (W-1)/2, the convention used throughout this repo.
constexpr int kImgW = 640;
constexpr int kImgH = 480;
constexpr double kHFov = 1.0472;

/// The intrinsics the URDF's wrist camera implies.
Intrinsics default_intrinsics();

/// Six coplanar points on a ring of radius r, in the panel frame.
Eigen::MatrixX3d hexagon(double r, double phi = 0.0);

/// Sort points counter-clockwise about their own centroid.
Eigen::MatrixX2d order_by_angle(const Eigen::Ref<const Eigen::MatrixX2d> & pts);

/// Rotate s_star to the cyclic shift that actually pairs with s.
///
/// Ordering six identical dots by angle fixes their ORDER but not where the
/// order starts, so a rotated hexagon can pair every dot with the wrong
/// target. The arm then chases a bogus goal: measured, one cell converged to
/// |e| 0.090 while its twin diverged to 0.530 running identical code. Picking
/// the shift with the smallest residual removes the ambiguity.
Eigen::MatrixX2d best_cyclic_match(
  const Eigen::Ref<const Eigen::MatrixX2d> & s,
  const Eigen::Ref<const Eigen::MatrixX2d> & s_star);

/// Stacked interaction matrix (2N, 6) at a single shared depth Z.
Eigen::MatrixXd interaction(const Eigen::Ref<const Eigen::MatrixX2d> & s, double Z);

/// Pseudo-inverse with singular directions below tau*s_max dropped.
///
/// tau belongs to the controller, never to the guard. Sharing one constant
/// between them was a real bug; see guard.hpp.
Eigen::MatrixXd pinv_trunc(const Eigen::Ref<const Eigen::MatrixXd> & L, double tau);

/// sqrt of the shoelace polygon area, floored at 1e-12 before the sqrt.
double poly_sigma(const Eigen::Ref<const Eigen::MatrixX2d> & s);

/// Camera twist. Partitioned when the partition is safe, plain otherwise.
Eigen::Matrix<double, 6, 1> ibvs_twist(
  const Eigen::Ref<const Eigen::MatrixX2d> & s,
  const Eigen::Ref<const Eigen::MatrixX2d> & s_star,
  bool use_partition);

}  // namespace arm_law
}  // namespace blindspot

#endif  // BLINDSPOT__ARM_LAW_HPP_
