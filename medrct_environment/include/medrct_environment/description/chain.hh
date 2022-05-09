#pragma once

#include <memory>
#include <string>
#include <vector>

#include "joint.hh"
#include "kinematics_tree.hh"

namespace medrct
{
namespace env
{
class Chain
{
public:
  Chain() = default;
  ~Chain() = default;
  using Ptr = std::shared_ptr<Chain>;
  using ConstPtr = std::shared_ptr<const Chain>;
  bool init(
      const KinematicsTree& kin_tree,
      const std::string& root_name,
      const std::string& tip_name);
  const std::vector<Joint::ConstPtr>& getRootToTipJoints() const;
  const std::vector<std::string>& getLinkNames() const;
  const std::vector<std::string>& getActiveJointNames() const;

private:
  // This vector is in order root to tip
  std::vector<Joint::ConstPtr> root_to_tip_joints;
  std::vector<std::string> link_names;
  std::vector<std::string> active_joint_names;
};
} // namespace env
} // namespace medrct
