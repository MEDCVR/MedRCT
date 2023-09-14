#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <medrct/types/types.hh>
#include <medrct/stream/stream.hh>

#include "controller.hh"
#include "controller_manager_config.hh"

namespace medrct
{
namespace controller
{

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
  std::unordered_map<std::string, std::shared_ptr<Controller>>
      name_to_controllers;
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
  std::shared_ptr<stream::SubStream<medrct::Joy>> clutch_subscriber;
  void clutchCallback(const medrct::Joy& joy);
  std::shared_ptr<stream::SubStream<medrct::Joy>> switch_subscriber;
  std::string switched_control_group_name;
  std::function<void()> switch_function;
  std::function<void()> unswitch_function;
  void switchCallback(const medrct::Joy& joy);
};
} // namespace controller
} // namespace medrct
