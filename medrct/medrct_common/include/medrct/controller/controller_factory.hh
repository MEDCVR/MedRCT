#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct/stream/stream_factory.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/controller/controller.hh>
namespace medrct
{

namespace controller
{

class ControllerFactory
{
public:
  using Ptr = std::shared_ptr<ControllerFactory>;
  using ConstPtr = std::shared_ptr<const ControllerFactory>;
  ControllerFactory() = default;
  virtual ~ControllerFactory() = default;
  virtual Controller::Ptr create(
      const medrct::stream::StreamFactory& stream_factory,
      YAML::Node config) const = 0;

protected:
  void getControllerName(
      std::string& name, const YAML::Node& controller_config) const;
};

} // namespace controller
} // namespace medrct
