#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct/stream/stream_factory.hh>
#include <medrct/controller/controller.hh>
#include <medrct/controller/controller_factory.hh>

namespace medrct
{
namespace controller
{

class DefaultControllerFactory : public ControllerFactory
{
public:
  DefaultControllerFactory() = default;
  virtual ~DefaultControllerFactory() = default;
  virtual Controller::Ptr create(
      const medrct::stream::StreamFactory& stream_factory,
      YAML::Node config) const override;
};

} // namespace controller
} // namespace medrct
