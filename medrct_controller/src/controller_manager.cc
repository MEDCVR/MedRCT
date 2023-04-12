#include <algorithm>
#include <utility>

#include <medrct/log.hh>
#include <medrct_controller/controller_manager.hh>

namespace medrct
{
namespace controller
{
ControllerManager::ControllerManager()
    : current_state(controller_state_t::UNINITIALIZED)
{

  // Instantiate Controllers here
}
ControllerManager::~ControllerManager()
{
}

ControllerManager::ControllerManager(ControllerManager&& other) noexcept
{
  current_state = other.current_state;
  active_control_group_name = std::move(other.active_control_group_name);
  name_to_control_groups = std::move(other.name_to_control_groups);
  name_to_controllers = std::move(other.name_to_controllers);
}

ControllerManager& ControllerManager::
operator=(ControllerManager&& other) noexcept
{
  current_state = other.current_state;
  active_control_group_name = std::move(other.active_control_group_name);
  name_to_control_groups = std::move(other.name_to_control_groups);
  name_to_controllers = std::move(other.name_to_controllers);
  return *this;
}

inline bool CMIsValidPreviousState(
    const controller_state_t destination_state,
    const controller_state_t current_state)
{
  if (!IsValidPreviousState(destination_state, current_state))
  {
    medrctlog::error(
        "Controller Manager: Current state [{}], not valid for transition to "
        "state [{}]",
        CONTROLLER_STATE_TO_STRING.at(current_state),
        CONTROLLER_STATE_TO_STRING.at(destination_state));
    return false;
  }
  return true;
}

bool ControllerManager::init(const ControllerManagerConfig& config)
{
  if (!CMIsValidPreviousState(controller_state_t::DISABLED, current_state))
  {
    return false;
  }

  if (config.controllers.size() == 0)
  {
    medrctlog::error("There must be at least one controler");
    return false;
  }
  std::set<std::string> controller_names;
  for (std::shared_ptr<Controller> controller : config.controllers)
  {
    std::string name = controller->getName();
    if (controller->getState() == controller_state_t::UNINITIALIZED)
    {
      medrctlog::error("controller {} must be initialized", name);
      return false;
    }
    if (name_to_controllers.find(name) != name_to_controllers.end())
    {
      medrctlog::error(
          "controller must have unique names. Non-unique name: {}", name);
      return false;
    }
    controller_names.insert(name);
    name_to_controllers.insert(
        std::make_pair<std::string, std::shared_ptr<Controller>>(
            std::move(name), std::move(controller)));
  }

  if (config.control_groups.size() == 0)
  {
    medrctlog::warn("No specified controller group, making all "
                    "controllers one group with name [default]. ");
    ControlGroup control_group;
    control_group.name = "default";
    control_group.controller_names = controller_names;
    control_group_names.push_back(control_group.name);
    name_to_control_groups[control_group.name] = control_group;
  }
  else
  {
    control_group_names.reserve(config.control_groups.size());
    for (const ControlGroup& control_group : config.control_groups)
    {
      if (name_to_control_groups.find(control_group.name) !=
          name_to_control_groups.end())
      {
        medrctlog::error(
            "control groups must have unique names. Non-unique name: {}",
            control_group.name);
        return false;
      }
      for (const std::string& controller_name : control_group.controller_names)
      {
        if (name_to_controllers.find(controller_name) ==
            name_to_controllers.end())
        {
          medrctlog::error(
              "No matching controller name in controllers for control group "
              "name {}",
              controller_name);
          return false;
        }
      }
      control_group_names.push_back(control_group.name);
      name_to_control_groups[control_group.name] = control_group;
    }
  }

  if (config.active_control_group_name != "")
  {
    active_control_group_name = active_control_group_name;
  }
  else
  {
    medrctlog::warn(
        "No active_control_group_name, will use first control group");
    active_control_group_name = control_group_names[0];
  }
  current_state = controller_state_t::DISABLED;
  return true;
}

bool ControllerManager::enable()
{
  if (!CMIsValidPreviousState(controller_state_t::ENABLED, current_state))
  {
    return false;
  }
  const ControlGroup& control_group =
      name_to_control_groups.at(active_control_group_name);
  for (const std::string& controller_name : control_group.controller_names)
  {
    if (!name_to_controllers.at(controller_name)->enable())
    {
      medrctlog::error("controller: {} failed to enable", controller_name);
      // TODO error processing?
      return false;
    }
  }
  current_state = controller_state_t::ENABLED;
  medrctlog::info(
      "Control Group [{}]: is now state [{}]",
      active_control_group_name,
      CONTROLLER_STATE_TO_STRING.at(current_state));
  return true;
}
bool ControllerManager::disable()
{
  if (!CMIsValidPreviousState(controller_state_t::DISABLED, current_state))
  {
    return false;
  }
  const ControlGroup& control_group =
      name_to_control_groups.at(active_control_group_name);
  for (const std::string& controller_name : control_group.controller_names)
  {
    if (!name_to_controllers.at(controller_name)->disable())
    {
      medrctlog::error("controller: {} failed to disable", controller_name);
      // TODO error processing?
      return false;
    }
  }
  current_state = controller_state_t::DISABLED;
  medrctlog::info(
      "Control Group [{}]: is now state [{}]",
      active_control_group_name,
      CONTROLLER_STATE_TO_STRING.at(current_state));
  return true;
}
bool ControllerManager::clutch()
{
  if (!CMIsValidPreviousState(controller_state_t::CLUTCHED, current_state))
  {
    return false;
  }
  const ControlGroup& control_group =
      name_to_control_groups.at(active_control_group_name);
  for (const std::string& controller_name : control_group.controller_names)
  {
    if (!name_to_controllers.at(controller_name)->clutch())
    {
      medrctlog::error("controller: {} failed to clutch", controller_name);
      // TODO error processing?
      return false;
    }
  }
  current_state = controller_state_t::CLUTCHED;
  medrctlog::info(
      "Control Group [{}]: is now state [{}]",
      active_control_group_name,
      CONTROLLER_STATE_TO_STRING.at(current_state));
  return true;
}
bool ControllerManager::unclutch()
{
  if (!CMIsValidPreviousState(controller_state_t::ENABLED, current_state))
  {
    return false;
  }
  const ControlGroup& control_group =
      name_to_control_groups.at(active_control_group_name);
  for (const std::string& controller_name : control_group.controller_names)
  {
    if (!name_to_controllers.at(controller_name)->unclutch())
    {
      medrctlog::error("controller: {} failed to unclutch", controller_name);
      // TODO error processing?
      return false;
    }
  }
  current_state = controller_state_t::ENABLED;
  medrctlog::info(
      "Control Group [{}]: is now state [{}]",
      active_control_group_name,
      CONTROLLER_STATE_TO_STRING.at(current_state));
  return true;
}

bool ControllerManager::setActiveControlGroup(const std::string& group_name)
{
  if (current_state == controller_state_t::UNINITIALIZED)
  {
    medrctlog::error("Controller manager must be initialized first");
    return false;
  }
  else if (
      name_to_control_groups.find(group_name) == name_to_control_groups.end())
  {
    medrctlog::error("Cannot find requested control group {}", group_name);
    return false;
  }
  else if (current_state == controller_state_t::CLUTCHED)
  {
    medrctlog::error(
        "Cannot set new active controler group while state is {}",
        CONTROLLER_STATE_TO_STRING.at(controller_state_t::CLUTCHED));
    // TODO: error_msg: Cannot set new controler group while clutched
    return false;
  }
  else if (current_state == controller_state_t::DISABLED)
  {
    active_control_group_name = group_name;
    medrctlog::info(
        "New Control Group is [{}]: and is state [{}]",
        active_control_group_name,
        CONTROLLER_STATE_TO_STRING.at(current_state));
    return true;
  }
  else if (current_state == controller_state_t::IN_ERROR)
  {
    medrctlog::error(
        "Cannot set new active controler group while state is {}",
        CONTROLLER_STATE_TO_STRING.at(controller_state_t::IN_ERROR));
    // TODO: error_msg: Controller is in error
    return false;
  }

  // else: current_state == controller_state_t::ENABLED
  // Activate other controllers while not deactivating same controllers,
  // iin previous control group
  const std::set<std::string>& current_group_names =
      name_to_control_groups.at(active_control_group_name).controller_names;
  const std::set<std::string>& new_group_names =
      name_to_control_groups.at(group_name).controller_names;

  std::vector<std::string> union_group;
  std::set_intersection(
      current_group_names.begin(),
      current_group_names.end(),
      new_group_names.begin(),
      new_group_names.end(),
      std::back_inserter(union_group));

  for (const auto& current_controller_name : current_group_names)
  {
    if (std::find(
            union_group.begin(), union_group.end(), current_controller_name) !=
        union_group.end())
    {
      continue;
    }
    if (!name_to_controllers.at(current_controller_name)->disable())
    {
      medrctlog::error(
          "controller: {} failed to disable while changing control group",
          current_controller_name);
      return false;
    }
  }

  for (const auto& new_controller_name : new_group_names)
  {
    if (std::find(
            union_group.begin(), union_group.end(), new_controller_name) !=
        union_group.end())
    {
      continue;
    }
    if (!name_to_controllers.at(new_controller_name)->enable())
    {
      medrctlog::error(
          "controller: {} failed to enable while changing control group",
          new_controller_name);
      return false;
    }
  }
  active_control_group_name = group_name;
  medrctlog::info(
      "New Control Group is [{}]: and is state [{}]",
      active_control_group_name,
      CONTROLLER_STATE_TO_STRING.at(current_state));
  return true;
}

std::string ControllerManager::getActiveControlGroupName() const
{
  return active_control_group_name;
}
const std::vector<std::string>& ControllerManager::getControlGroupNames() const
{
  return {control_group_names};
}
controller_state_t ControllerManager::getCurrentState() const
{
  return current_state;
}

BasicControllerManagerCommunicator::BasicControllerManagerCommunicator()
{
}
BasicControllerManagerCommunicator::~BasicControllerManagerCommunicator()
{
}

bool BasicControllerManagerCommunicator::init(
    ControllerManager&& cm,
    const BasicControllerManagerCommunicatorConfig& config)
{
  // Only we will use this, and people online say not to do so rigorous checks
  // if sure that not third party clients will use, but this is only for
  // initilization so it's ok.
  if (cm.getCurrentState() == controller_state_t::UNINITIALIZED)
  {
    medrctlog::error("controller manager is not initialized");
    return false;
  }
  if (!config.clutch_subscriber)
  {
    medrctlog::error("clutch_subscriber is a null ptr");
    return false;
  }
  clutch_subscriber = config.clutch_subscriber;
  if (!config.switch_subscriber)
  {
    medrctlog::error("switch_subscriber is a null ptr");
    return false;
  }
  switch_subscriber = config.switch_subscriber;

  if (config.active_control_group_name != "")
  {
    if (!cm.setActiveControlGroup(active_control_group_name))
    {
      return false;
    }
    active_control_group_name = config.active_control_group_name;
  }
  else
  {
    active_control_group_name = cm.getActiveControlGroupName();
  }

  if (config.switched_control_group_name != "")
  {
    const auto& control_group_names = cm.getControlGroupNames();
    if (std::find(
            control_group_names.begin(),
            control_group_names.end(),
            config.switched_control_group_name) == control_group_names.end())
    {
      medrctlog::error("switched_control_group_name not in controller "
                       "manager control groups");
      return false;
    }
    switched_control_group_name = config.switched_control_group_name;
    switch_function = [this]() {
      controller_manager.setActiveControlGroup(switched_control_group_name);
    };
    unswitch_function = [this]() {
      controller_manager.setActiveControlGroup(active_control_group_name);
    };
  }
  if (config.auto_enable)
  {
    if (!cm.enable())
    {
      medrctlog::error("controller manager failed to enable");
      return false;
    }
  }
  controller_manager = std::move(cm);
  clutch_subscriber->addCallback(
      "clutch_callback",
      std::bind(
          &BasicControllerManagerCommunicator::clutchCallback,
          this,
          std::placeholders::_1));
  switch_subscriber->addCallback(
      "switch_callback",
      std::bind(
          &BasicControllerManagerCommunicator::switchCallback,
          this,
          std::placeholders::_1));
  return true;
}

void BasicControllerManagerCommunicator::clutchCallback(const medrct::Joy& joy)
{
  if (joy.buttons.size() > 0)
  {
    if (joy.buttons[0] == 1)
    {
      controller_manager.clutch();
    }
    else if (joy.buttons[0] == 0)
    {
      controller_manager.unclutch();
    }
    else
    {
      medrctlog::warn("clutchCallback: button[0] value should be 1 or 0. Not "
                      "doing anything");
    }
  }
  else
  {
    medrctlog::warn(
        "clutchCallback: button vector should be size of 1 or more. Not "
        "doing anything");
  }
  return;
}

void BasicControllerManagerCommunicator::switchCallback(const medrct::Joy& joy)
{
  if (joy.buttons.size() > 0)
  {
    if (joy.buttons[0] == 1)
    {
      switch_function();
    }
    else if (joy.buttons[0] == 0)
    {
      unswitch_function();
    }
    else
    {
      medrctlog::warn("switchCallback: button[0] value should be 1 or 0. Not "
                      "doing anything");
    }
  }
  else
  {
    medrctlog::warn(
        "switchCallback: button vector should be size of 1 or more. Not "
        "doing anything");
  }
  return;
}
} // namespace controller
} // namespace medrct
