#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct/stream/stream_factory.hh>

namespace medrct
{
namespace stream
{

class Ros2StreamFactory : public StreamFactory
{
public:
  using Ptr = std::shared_ptr<Ros2StreamFactory>;
  using ConstPtr = std::shared_ptr<const Ros2StreamFactory>;
  Ros2StreamFactory() = default;
  virtual ~Ros2StreamFactory() = default;

  // Should throw if nullptr
  virtual Stream::Ptr create(const YAML::Node& config) const final;
};
} // namespace stream
} // namespace medrct
