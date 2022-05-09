#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <medrct_common/types.hh>
#include <medrct_common/interface/stream.hh>

#include "controller.hh"

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
  std::vector<std::shared_ptr<ControllerInterface>> controllers;
  // (Optional) If left empty, will be the first control group in
  // control_groups.
  std::string active_control_group_name = "";
};

class ControllerManager
{
public:
  ControllerManager();
  ~ControllerManager();
  ControllerManager(const ControllerManager& other) = delete;
  ControllerManager& operator=(const ControllerManager& other) = delete;
  ControllerManager(ControllerManager&& other) noexcept;
  ControllerManager& operator=(ControllerManager&& other) noexcept;

  bool init(const ControllerManagerConfig& config);
  // All these methods work on the getActiveControlGroupName();
  bool enable();
  bool disable();
  bool clutch();
  bool unclutch();
  bool setActiveControlGroup(const std::string& group_name);
  std::string getActiveControlGroupName() const;
  const std::vector<std::string>& getControlGroupNames() const;
  controller_state_t getCurrentState() const;

private:
  controller_state_t current_state;
  std::string active_control_group_name;
  std::vector<std::string> control_group_names;
  std::unordered_map<std::string, ControlGroup> name_to_control_groups;
  std::unordered_map<std::string, std::shared_ptr<ControllerInterface>>
      name_to_controllers;
};

struct BasicControllerManagerCommunicatorConfig
{
  std::shared_ptr<stream::InputStream<medrct::Joy>> clutch_subscriber;
  std::shared_ptr<stream::InputStream<medrct::Joy>> switch_subscriber;
  std::string active_control_group_name;
  std::string switched_control_group_name = "";
  bool auto_enable = false;
};

class BasicControllerManagerCommunicator
{
public:
  BasicControllerManagerCommunicator();
  virtual ~BasicControllerManagerCommunicator();
  bool init(
      ControllerManager&& cm,
      const BasicControllerManagerCommunicatorConfig& config);

private:
  ControllerManager controller_manager;
  std::string active_control_group_name;
  std::shared_ptr<stream::InputStream<medrct::Joy>> clutch_subscriber;
  void clutchCallback(const medrct::Joy& joy);
  std::shared_ptr<stream::InputStream<medrct::Joy>> switch_subscriber;
  std::string switched_control_group_name;
  std::function<void()> switch_function;
  std::function<void()> unswitch_function;
  void switchCallback(const medrct::Joy& joy);
};
} // namespace controller
} // namespace medrct
