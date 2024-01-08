#pragma once

#include <yaml-cpp/yaml.h>

#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/kinematics/kinematics_factory.hh>

namespace medrct
{
namespace env
{
class FrankaForwardKinematicsFactory : public ForwardKinematicsFactory
{
public:
  FrankaForwardKinematicsFactory() = default;
  std::shared_ptr<ForwardKinematics>
  create(const KinematicsTree::Ptr, const YAML::Node& config) final;
};

class FrankaInverseKinematicsFactory : public InverseKinematicsFactory
{
public:
  FrankaInverseKinematicsFactory() = default;
  std::shared_ptr<InverseKinematics>
  create(const KinematicsTree::Ptr, const YAML::Node& config) final;
};
} // namespace env
} // namespace medrct
