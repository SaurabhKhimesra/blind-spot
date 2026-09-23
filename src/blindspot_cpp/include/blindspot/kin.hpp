// Forward kinematics and geometric Jacobian for a serial chain, from URDF.
//
// Eigen and urdfdom, no KDL. Written so it can be checked against TF rather
// than trusted: comparing fk() to what robot_state_publisher reports is how a
// wrong axis or a missed fixed joint shows up.
#ifndef BLINDSPOT__KIN_HPP_
#define BLINDSPOT__KIN_HPP_

#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace blindspot
{

Eigen::Matrix3d rpy_to_R(double r, double p, double y);

Eigen::Matrix4d T_of(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy);

/// Rodrigues rotation of angle q about `axis`, which is normalised first.
Eigen::Matrix3d axis_R(const Eigen::Vector3d & axis, double q);

/// Chain from `base` to `tip`, with the movable joints in order.
class Chain
{
public:
  /// Throws std::runtime_error if the URDF will not parse or there is no
  /// path from `tip` up to `base`.
  Chain(const std::string & urdf_xml, const std::string & base, const std::string & tip);

  const std::vector<std::string> & joint_names() const {return joint_names_;}

  /// A joint axis in the base frame, with the point it passes through.
  struct Axis
  {
    Eigen::Vector3d z;
    Eigen::Vector3d p;
  };

  /// (T_base_tip, joint axes in the base frame).
  std::pair<Eigen::Matrix4d, std::vector<Axis>> fk_all(
    const Eigen::Ref<const Eigen::VectorXd> & q) const;

  Eigen::Matrix4d fk(const Eigen::Ref<const Eigen::VectorXd> & q) const;

  /// Geometric Jacobian of the tip, expressed in the base frame. (6, n).
  Eigen::MatrixXd jacobian(const Eigen::Ref<const Eigen::VectorXd> & q) const;

private:
  struct Step
  {
    Eigen::Matrix4d T_origin;
    Eigen::Vector3d axis;
    bool movable{false};
  };

  std::vector<Step> steps_;
  std::vector<std::string> joint_names_;
};

}  // namespace blindspot

#endif  // BLINDSPOT__KIN_HPP_
