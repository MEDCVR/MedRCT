#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct_common/interface/stream_factory.hh>

namespace medrct
{
namespace stream
{

class RosStreamFactory : public StreamFactory
{
public:
  using Ptr = std::shared_ptr<RosStreamFactory>;
  using ConstPtr = std::shared_ptr<const RosStreamFactory>;
  RosStreamFactory() = default;
  virtual ~RosStreamFactory() = default;

  // Should throw if nullptr
  virtual Stream::Ptr create(const YAML::Node& config) const final;
};
} // namespace stream
} // namespace medrct
