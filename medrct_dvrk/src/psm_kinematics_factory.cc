#include <medrct_common/log.hh>
#include <medrct_common/loader/class_loader.hh>
#include <medrct_dvrk/psm_kinematics_solver.hh>
#include <medrct_dvrk/psm_tool_params.hh>

#include <medrct_dvrk/psm_kinematics_factory.hh>

namespace medrct
{
namespace env
{
std::shared_ptr<PsmKinematicsData>
parsePsmKinematicsData(const YAML::Node& config)
{
  std::string tool_name;
  if (YAML::Node n = config["tool_name"])
    tool_name = n.as<std::string>();
  else
    throw std::runtime_error("No parameter [tool_name] in config.");

  auto psm_kin_data = std::make_shared<PsmKinematicsData>();
  if (tool_name == "LND400006")
    psm_kin_data->init(LND400006());
  else
    throw std::runtime_error("No valid tool_name: " + tool_name + ".");
  return psm_kin_data;
}

std::shared_ptr<ForwardKinematics> PsmForwardKinematicsFactory::create(
    const KinematicsTree&, const YAML::Node& config)
{
  std::shared_ptr<PsmKinematicsData> psm_kin_data;
  try
  {
    psm_kin_data = parsePsmKinematicsData(config);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "PsmForwardKinematicsFactory: failed to parse yaml config. " +
        std::string(e.what()));
    return nullptr;
  }
  return std::make_shared<PsmForwardKinematics>(*psm_kin_data);
}
std::shared_ptr<InverseKinematics> PsmInverseKinematicsFactory::create(
    const KinematicsTree&, const YAML::Node& config)
{
  std::shared_ptr<PsmKinematicsData> psm_kin_data;
  try
  {
    psm_kin_data = parsePsmKinematicsData(config);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "PsmInverseKinematicsFactory: failed to parse yaml config. " +
        std::string(e.what()));
    return nullptr;
  }
  return std::make_shared<PsmInverseKinematics>(*psm_kin_data);
}
} // namespace env
} // namespace medrct

// clang-format off
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::env::PsmForwardKinematicsFactory, PsmForwardKinematicsFactory, FwdKin)
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::env::PsmInverseKinematicsFactory, PsmInverseKinematicsFactory, InvKin)
// clang-format on
