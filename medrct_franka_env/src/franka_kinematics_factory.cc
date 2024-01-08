#include <medrct/log.hh>
#include <medrct/loader/class_loader.hh>
#include <medrct_franka_env/franka_kinematics_solver.hh>
#include <medrct_franka_env/franka_kinematics_factory.hh>

namespace medrct
{
namespace env
{
// std::shared_ptr<PsmKinematicsData>
// parsePsmKinematicsData(const YAML::Node& config)
// {
//   std::string tool_name;
//   if (YAML::Node n = config["tool_name"])
//     tool_name = n.as<std::string>();
//   else
//     throw std::runtime_error("No parameter [tool_name] in config.");

//   auto psm_kin_data = std::make_shared<PsmKinematicsData>();
//   if (tool_name == "LND400006")
//     psm_kin_data->init(LND400006());
//   else
//     throw std::runtime_error("No valid tool_name: " + tool_name + ".");
//   return psm_kin_data;
// }

std::shared_ptr<ForwardKinematics> FrankaForwardKinematicsFactory::create(
    const KinematicsTree::Ptr, const YAML::Node& config)
{
  return std::make_shared<FrankaForwardKinematics>();
}
std::shared_ptr<InverseKinematics> FrankaInverseKinematicsFactory::create(
    const KinematicsTree::Ptr, const YAML::Node& config)
{
  return std::make_shared<FrankaInverseKinematics>();
}
} // namespace env
} // namespace medrct

// clang-format off
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::env::FrankaForwardKinematicsFactory, FrankaForwardKinematicsFactory, FwdKin)
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::env::FrankaInverseKinematicsFactory, FrankaInverseKinematicsFactory, InvKin)
// clang-format on
