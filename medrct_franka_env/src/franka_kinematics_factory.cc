#include <medrct/log.hh>
#include <medrct/loader/class_loader.hh>
#include <medrct_franka_env/franka_kinematics_solver.hh>
#include <medrct_franka_env/franka_kinematics_factory.hh>

namespace medrct
{
namespace env
{

std::shared_ptr<ForwardKinematics> FrankaForwardKinematicsFactory::create(
    const KinematicsTree::Ptr, const YAML::Node&)
{
  return std::make_shared<FrankaForwardKinematics>();
}
std::shared_ptr<InverseKinematics> FrankaInverseKinematicsFactory::create(
    const KinematicsTree::Ptr, const YAML::Node&)
{
  return std::make_shared<FrankaInverseKinematics>();
}
} // namespace env
} // namespace medrct

// clang-format off
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::env::FrankaForwardKinematicsFactory, FrankaForwardKinematicsFactory, FwdKin)
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::env::FrankaInverseKinematicsFactory, FrankaInverseKinematicsFactory, InvKin)
// clang-format on
