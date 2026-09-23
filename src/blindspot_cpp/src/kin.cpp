#include "blindspot/kin.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>

#include <urdf/model.hpp>

namespace blindspot
{

Eigen::Matrix3d rpy_to_R(double r, double p, double y)
{
  const double cr = std::cos(r);
  const double sr = std::sin(r);
  const double cp = std::cos(p);
  const double sp = std::sin(p);
  const double cy = std::cos(y);
  const double sy = std::sin(y);
  Eigen::Matrix3d R;
  R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr;
  return R;
}

Eigen::Matrix4d T_of(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() = rpy_to_R(rpy(0), rpy(1), rpy(2));
  T.topRightCorner<3, 1>() = xyz;
  return T;
}

Eigen::Matrix3d axis_R(const Eigen::Vector3d & axis, double q)
{
  const Eigen::Vector3d a = axis.normalized();
  Eigen::Matrix3d K;
  K << 0.0, -a(2), a(1),
    a(2), 0.0, -a(0),
    -a(1), a(0), 0.0;
  return Eigen::Matrix3d::Identity() + std::sin(q) * K + (1.0 - std::cos(q)) * (K * K);
}

Chain::Chain(const std::string & urdf_xml, const std::string & base, const std::string & tip)
{
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("could not parse the URDF");
  }

  std::map<std::string, urdf::JointConstSharedPtr> parent_joint;
  for (const auto & [name, joint] : model.joints_) {
    (void)name;
    parent_joint[joint->child_link_name] = joint;
  }

  // Walk up from the tip, then replay the chain base-first.
  std::vector<urdf::JointConstSharedPtr> chain;
  std::string node = tip;
  while (node != base) {
    const auto it = parent_joint.find(node);
    if (it == parent_joint.end()) {
      throw std::runtime_error(
              "no path from " + tip + " up to " + base + " (stuck at " + node + ")");
    }
    chain.push_back(it->second);
    node = it->second->parent_link_name;
  }

  for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
    const auto & j = *it;
    const auto & origin = j->parent_to_joint_origin_transform;
    double r = 0.0;
    double p = 0.0;
    double y = 0.0;
    // urdfdom stores the origin as a quaternion; rpy is what the URDF wrote,
    // so go back through it rather than composing a second representation.
    origin.rotation.getRPY(r, p, y);
    const Eigen::Vector3d xyz(origin.position.x, origin.position.y, origin.position.z);
    const Eigen::Matrix4d T = T_of(xyz, Eigen::Vector3d(r, p, y));

    Step step;
    step.T_origin = T;
    if (j->type == urdf::Joint::REVOLUTE || j->type == urdf::Joint::CONTINUOUS) {
      step.axis = Eigen::Vector3d(j->axis.x, j->axis.y, j->axis.z);
      step.movable = true;
      joint_names_.push_back(j->name);
    } else {
      step.movable = false;
    }
    steps_.push_back(step);
  }
}

std::pair<Eigen::Matrix4d, std::vector<Chain::Axis>> Chain::fk_all(
  const Eigen::Ref<const Eigen::VectorXd> & q) const
{
  if (q.size() < static_cast<Eigen::Index>(joint_names_.size())) {
    throw std::invalid_argument("fk_all: too few joint positions for this chain");
  }
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  std::vector<Axis> axes;
  Eigen::Index k = 0;
  for (const Step & step : steps_) {
    T = T * step.T_origin;
    if (step.movable) {
      Axis a;
      a.z = T.topLeftCorner<3, 3>() * step.axis.normalized();
      a.p = T.topRightCorner<3, 1>();
      axes.push_back(a);
      Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
      R.topLeftCorner<3, 3>() = axis_R(step.axis, q(k));
      T = T * R;
      ++k;
    }
  }
  return {T, axes};
}

Eigen::Matrix4d Chain::fk(const Eigen::Ref<const Eigen::VectorXd> & q) const
{
  return fk_all(q).first;
}

Eigen::MatrixXd Chain::jacobian(const Eigen::Ref<const Eigen::VectorXd> & q) const
{
  const auto [T, axes] = fk_all(q);
  const Eigen::Vector3d p_tip = T.topRightCorner<3, 1>();
  Eigen::MatrixXd J(6, static_cast<Eigen::Index>(axes.size()));
  for (size_t i = 0; i < axes.size(); ++i) {
    const Eigen::Index c = static_cast<Eigen::Index>(i);
    J.block<3, 1>(0, c) = axes[i].z.cross(p_tip - axes[i].p);
    J.block<3, 1>(3, c) = axes[i].z;
  }
  return J;
}

}  // namespace blindspot
