#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct/stream/stream_factory.hh>

namespace medrct
{
namespace stream
{

class IntraStreamFactory : public StreamFactory
{
public:
  virtual ~IntraStreamFactory() = default;

  // Should throw if nullptr
  virtual Stream::Ptr create(const YAML::Node& config) const final;
};
} // namespace stream
} // namespace medrct
