#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>

namespace medrct
{
namespace env
{
class ForwardKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<ForwardKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const ForwardKinematicsFactory>;
  virtual ~ForwardKinematicsFactory() = default;
  virtual std::shared_ptr<ForwardKinematics>
  create(const KinematicsTree::Ptr tree, const YAML::Node& config) = 0;
};

class InverseKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsFactory>;
  virtual ~InverseKinematicsFactory() = default;
  virtual std::shared_ptr<InverseKinematics>
  create(const KinematicsTree::Ptr tree, const YAML::Node& config) = 0;
};

} // namespace env
} // namespace medrct
