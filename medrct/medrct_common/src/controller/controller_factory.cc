#include <string>

#include <medrct/loader/class_loader.hh>
#include <medrct/log.hh>
#include <medrct/controller/controller_factory.hh>

namespace medrct
{
namespace controller
{

void ControllerFactory::getControllerName(
    std::string& name, const YAML::Node& controller_config) const
{
  if (YAML::Node n = controller_config["name"])
    name = n.as<std::string>();
  else
    throw std::runtime_error("No [name] in controller config.");
  return;
}

} // namespace controller
} // namespace medrct
