#include <string>

#include <medrct/types/joint_state.hh>
#include <medrct/log.hh>
#include <medrct/config.hh>

#include <medrct_default_controller/config.hh>
#include <medrct_default_controller/default_controller_factory.hh>
#include <medrct_default_controller/cartesian_teleop_controller.hh>
#include <medrct_default_controller/joint_teleop_controller.hh>

namespace medrct
{
namespace controller
{

Controller::Ptr DefaultControllerFactory::create(
    const medrct::stream::StreamFactory& stream_factory,
    YAML::Node controller_config) const
{
  std::string class_name = GetValue<std::string>(GetYamlNode(controller_config, "loader"), "create_class_name");
  std::string controller_name = GetValue<std::string>(controller_config, "name");

  YAML::Node input_config = GetYamlNode(controller_config, "input");
  if (class_name == "JointMimicController" ||
      class_name == "JointIncrementController")
  {
    JointTeleopControllerConfig jtcc;
    JointTeleopControllerConfig::FromYaml(jtcc, controller_config, stream_factory);
    if (class_name == "JointMimicController")
    {
      auto jmc = std::make_shared<JointMimicController>();
      if (!jmc->init(jtcc))
        throw std::invalid_argument(
            "Failed to init JointMimicController with name: " +
            jtcc.controller_name);
      return jmc;
    }
    // else Must be JointIncrementController
    auto jic = std::make_shared<JointIncrementController>();
    if (!jic->init(jtcc))
      throw std::invalid_argument(
          "Failed to init JointIncrementController with name: " +
          jtcc.controller_name);
    return jic;
  }

  if (class_name == "CartesianIncrementController")
  {
    CartesianIncrementControllerConfig cic_cfg;
    CartesianIncrementControllerConfig::FromYaml(cic_cfg, controller_config, stream_factory);
    Transform tf;
    cic_cfg.forward_kinematics->computeFK(tf, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto cic = std::make_shared<CartesianIncrementController>();
    if (!cic->init(cic_cfg))
      throw std::invalid_argument(
          "Failed to init CartesianIncrementController with name: " +
          cic_cfg.controller_name);
    return cic;
  }
  else if (class_name == "CartesianFollowerController")
  {
    CartesianFollowerControllerConfig cfcc;
    CartesianFollowerControllerConfig::FromYaml(cfcc, controller_config, stream_factory);
    auto cfc = std::make_shared<CartesianFollowerController>();
    if (!cfc->init(cfcc))
      throw std::invalid_argument(
          "Failed to init CartesianFollowerController with name: " +
          cfcc.controller_name);
    return cfc;
  }
  throw std::invalid_argument(
      "No valid controller specified with [create_class_name]: " + class_name);
  return nullptr;
}

} // namespace controller
} // namespace medrct

// clang-format off
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::controller::DefaultControllerFactory, DefaultControllerFactory, DCtrlF)
// clang-format on
