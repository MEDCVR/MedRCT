#pragma once

#include <yaml-cpp/yaml.h>

#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/kinematics/kinematics_factory.hh>

namespace medrct
{
namespace env
{
class PsmForwardKinematicsFactory : public ForwardKinematicsFactory
{
public:
  PsmForwardKinematicsFactory() = default;
  std::shared_ptr<ForwardKinematics>
  create(const KinematicsTree&, const YAML::Node& config) final;
};

class PsmInverseKinematicsFactory : public InverseKinematicsFactory
{
public:
  PsmInverseKinematicsFactory() = default;
  std::shared_ptr<InverseKinematics>
  create(const KinematicsTree&, const YAML::Node& config) final;
};
} // namespace env
} // namespace medrct
