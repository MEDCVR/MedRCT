#pragma once

#include <yaml-cpp/yaml.h>

namespace medrct
{
namespace controller
{

inline YAML::Node
GetYamlNode(const YAML::Node& yaml_node, const std::string& key)
{
  if (!yaml_node[key])
    throw std::invalid_argument("No [" + key + "] in config.");
  return yaml_node[key];
}

template <typename T>
T GetValue(const YAML::Node& yaml_node, const std::string& key)
{
  if (!yaml_node[key])
    throw std::invalid_argument("No [" + key + "] in config.");
  return yaml_node[key].as<T>();
}

template <typename T>
T GetValueDefault(
    const YAML::Node& yaml_node, const std::string& key, const T& default_value)
{
  if (!yaml_node[key])
    return default_value;
  return yaml_node[key].as<T>();
}

} // namespace controller
} // namespace medrct
