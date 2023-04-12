#include <algorithm>

#include <medrct/log.hh>
#include <medrct_controller/controller.hh>

namespace medrct
{
namespace controller
{

inline bool CIsValidPreviousState(
    const std::string& controller_name,
    const controller_state_t destination_state,
    const controller_state_t current_state)
{
  if (!IsValidPreviousState(destination_state, current_state))
  {
    medrctlog::error(
        "Controller {}: Current state [{}], not valid for transition to "
        "state [{}]",
        controller_name,
        CONTROLLER_STATE_TO_STRING.at(current_state),
        CONTROLLER_STATE_TO_STRING.at(destination_state));
    return false;
  }
  return true;
}

Controller::Controller() : current_state(controller_state_t::UNINITIALIZED)
{
}
Controller::~Controller()
{
}
bool Controller::init(const std::string& controller_name)
{
  if (controller_name == "")
  {
    medrctlog::error("controller_name must not be empty");
    return false;
  }
  name = controller_name;

  if (!CIsValidPreviousState(name, controller_state_t::DISABLED, current_state))
  {
    return false;
  }
  return true;
}
bool Controller::enable()
{
  if (!CIsValidPreviousState(name, controller_state_t::ENABLED, current_state))
  {
    return false;
  }
  if (!onEnable())
  {
    return false;
  }
  task->enable();
  current_state = controller_state_t::ENABLED;
  return true;
}
bool Controller::disable()
{
  if (!CIsValidPreviousState(name, controller_state_t::DISABLED, current_state))
  {
    return false;
  }
  if (!onDisable())
  {
    // TODO Should go to error processing state
    return false;
  };
  task->disable();
  current_state = controller_state_t::DISABLED;
  return true;
}
bool Controller::clutch()
{
  if (!CIsValidPreviousState(name, controller_state_t::CLUTCHED, current_state))
  {
    return false;
  }
  if (!onClutch())
  {
    return false;
  }
  task->disable();
  current_state = controller_state_t::CLUTCHED;
  return true;
}
bool Controller::unclutch()
{
  if (!CIsValidPreviousState(name, controller_state_t::ENABLED, current_state))
  {
    return false;
  }
  if (!onUnclutch())
  {
    return false;
  }
  task->enable();
  current_state = controller_state_t::ENABLED;
  return true;
}
controller_state_t Controller::getState() const
{
  return current_state;
}
std::string Controller::getName() const
{
  return name;
}
bool Controller::onEnable()
{
  return true;
}
bool Controller::onDisable()
{
  return true;
}
bool Controller::onClutch()
{
  return true;
}
bool Controller::onUnclutch()
{
  return true;
}
void Controller::updateProcess(DataStore& input_data)
{
  input_stream_map.getDataFromAllBufferedStreams(input_data);
  update(input_data);
}
} // namespace controller
} // namespace medrct
