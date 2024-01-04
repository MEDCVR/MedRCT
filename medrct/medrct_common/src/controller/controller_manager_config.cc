#include <algorithm>
#include <utility>

#include <medrct/loader/class_loader.hh>
#include <medrct/log.hh>
#include <medrct/controller/controller_manager_config.hh>

namespace medrct
{
namespace controller
{
ControlGroup FromYAMLControlGroup(const YAML::Node& control_group_cfg)
{
  ControlGroup control_group;
  if (YAML::Node n = control_group_cfg["name"])
  {
    control_group.name = n.as<std::string>();
    if (YAML::Node controller_names_cfg = control_group_cfg["controller_names"])
    {
      for (YAML::const_iterator it = controller_names_cfg.begin();
           it != controller_names_cfg.end();
           ++it)
      {
        control_group.controller_names.emplace(it->as<std::string>());
      }
    }
    else
    {
      throw std::runtime_error(
          "No [controller_names] key in control group config");
    }
  }
  else
  {
    throw std::runtime_error("No [name] key in control group config");
  }

  if (control_group.controller_names.size() == 0)
  {
    throw std::runtime_error(
        "Control group should have at least one controller");
  }
  return control_group;
}

ControllerManagerConfig FromYAMLControllerManagerConfig(
    const YAML::Node& config, const medrct::stream::StreamFactory& sf)
{
  const YAML::Node controllers_config = config["controllers"];
  if (!controllers_config)
  {
    throw std::invalid_argument(
        "ControllerManagerConfig: No [controllers] key in config");
  }
  ControllerManagerConfig cmc;
  for (YAML::const_iterator it = controllers_config.begin();
       it != controllers_config.end();
       ++it)
  {
    const YAML::Node& controller_cfg = *it;
    controller::ControllerFactory::Ptr controller_factory;
    try
    {
      controller_factory =
          medrct_loader::createSharedInstance<controller::ControllerFactory>(
              controller_cfg["loader"]);
      cmc.controllers.emplace_back(
          controller_factory->create(sf, controller_cfg));
    }
    catch (const std::exception& e)
    {
      throw std::invalid_argument(
          "ControllerManagerConfig: Controller factory failed to load: " +
          std::string(e.what()));
    }
  }

  if (const YAML::Node control_groups_cfg = config["control_groups"])
  {
    for (YAML::const_iterator it = control_groups_cfg.begin();
         it != control_groups_cfg.end();
         ++it)
    {
      cmc.control_groups.emplace_back(FromYAMLControlGroup(*it));
    }
  }

  if (const YAML::Node n = config["active_control_group_name"])
  {
    cmc.active_control_group_name = n.as<std::string>();
  }
  return cmc;
}

BasicControllerManagerCommunicatorConfig
FromYAMLBasicControllerCommunicatorConfig(
    const YAML::Node& config, const medrct::stream::StreamFactory& sf)
{
  BasicControllerManagerCommunicatorConfig bcmcc;
  if (const YAML::Node n = config["active_control_group_name"])
  {
    bcmcc.active_control_group_name = n.as<std::string>();
  }
  if (YAML::Node n = config["clutch_topic"])
  {
    n["name"] = "clutch_topic_input_stream";
    n["type"] = "input";
    n["data_type"] = "Joy";
    bcmcc.clutch_subscriber = sf.create<stream::SubStream<medrct::Joy>>(n);
  }

  if (YAML::Node n = config["enable_topic"])
  {
    n["name"] = "enable_topic_input_stream";
    n["type"] = "input";
    n["data_type"] = "Joy";
    bcmcc.switch_subscriber = sf.create<stream::SubStream<medrct::Joy>>(n);
  }

  if (YAML::Node n = config["switched_control_group_name"])
  {
    bcmcc.switched_control_group_name = n.as<std::string>();
  }
  if (YAML::Node n = config["auto_enable"])
  {
    bcmcc.auto_enable = n.as<bool>();
  }
  return bcmcc;
}

} // namespace controller
} // namespace medrct
