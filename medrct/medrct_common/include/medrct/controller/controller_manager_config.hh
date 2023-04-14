#pragma once

#include <vector>
#include <medrct/types/types.hh>
#include <medrct/stream/stream.hh>
#include <medrct/stream/stream_factory.hh>
#include "controller_factory.hh"

namespace medrct
{
namespace controller
{

struct ControlGroup
{
  std::string name;
  std::set<std::string> controller_names;
};

struct ControllerManagerConfig
{
  // Each control group must have a unique name
  // The controller_names must be a valid name in controllers
  std::vector<ControlGroup> control_groups;
  // Controllers must have been initialized beforehand
  std::vector<std::shared_ptr<Controller>> controllers;
  // (Optional) If left empty, will be the first control group in
  // control_groups.
  std::string active_control_group_name = "";
};

struct BasicControllerManagerCommunicatorConfig
{
  std::shared_ptr<stream::InputStream<medrct::Joy>> clutch_subscriber;
  std::shared_ptr<stream::InputStream<medrct::Joy>> switch_subscriber;
  std::string active_control_group_name;
  std::string switched_control_group_name = "";
  bool auto_enable = false;
};

ControlGroup FromYAMLControlGroup(const YAML::Node& control_group_cfg);
ControllerManagerConfig FromYAMLControllerManagerConfig(
    const YAML::Node& config, const medrct::stream::StreamFactory& sf);
BasicControllerManagerCommunicatorConfig
FromYAMLBasicControllerCommunicatorConfig(
    const YAML::Node& config, const medrct::stream::StreamFactory& sf);

} // namespace controller
} // namespace medrct
