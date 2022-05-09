#pragma once
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include <medrct_common/intra/intra_stream.hh>
#include <medrct_common/joint_state.hh>
#include <medrct_common/types.hh>
#include <medrct_common/twist.hh>

namespace medrct
{
namespace stream
{

template <template <typename> class T>
Stream::Ptr createStreamWithDataType(
    const std::string& topic_name,
    const std::string& name,
    const std::string& data_type)
{
  // Is there a better way to do this?
  if (data_type == "JointState")
  {
    return std::make_shared<T<JointState>>(topic_name, name);
  }
  else if (data_type == "Transform")
  {
    return std::make_shared<T<Transform>>(topic_name, name);
  }
  else if (data_type == "Twist")
  {
    return std::make_shared<T<Twist>>(topic_name, name);
  }
  throw std::runtime_error("Data type not supported yet");
  return nullptr;
}

} // namespace stream
} // namespace medrct
