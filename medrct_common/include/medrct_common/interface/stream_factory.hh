#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct_common/interface/stream.hh>

namespace medrct
{
namespace stream
{
class StreamFactory
{
public:
  using Ptr = std::shared_ptr<StreamFactory>;
  using ConstPtr = std::shared_ptr<const StreamFactory>;
  virtual ~StreamFactory() = default;

  // Should throw if nullptr
  virtual Stream::Ptr create(const YAML::Node& config) const = 0;

  template <class T>
  std::shared_ptr<T> create(const YAML::Node& config) const
  {
    std::shared_ptr<T> casted = std::dynamic_pointer_cast<T>(create(config));
    if (!casted)
    {
      throw std::runtime_error("Created stream was not of casted type");
    }
    return casted;
  }
};
} // namespace stream
} // namespace medrct
