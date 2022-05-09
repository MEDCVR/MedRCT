#include <algorithm>

#include <medrct_common/log.hh>
#include <medrct_controller/controller.hh>

namespace medrct
{
namespace controller
{

using namespace medrct::stream;

DataStore::DataStore()
{
}
DataStore::~DataStore()
{
}

StreamMap::StreamMap()
{
}
StreamMap::~StreamMap()
{
}
void StreamMap::add(const std::shared_ptr<Stream> stream)
{
  std::string name = stream->name;
  name_to_streams[name] = stream;
}
bool StreamMap::removeBuffer(const std::string& stream_name)
{
  auto it = buffered_stream_name_to_functions.find(stream_name);
  if (it == buffered_stream_name_to_functions.end())
  {
    return false;
  }
  it->second.remove_buffer_func();
  buffered_stream_name_to_functions.erase(it);
  return true;
}

bool StreamMap::waitForOneBufferedDataInput(const std::string& name, bool print)
{
  auto it = buffered_stream_name_to_functions.find(name);
  if (it == buffered_stream_name_to_functions.end())
  {
    throw std::logic_error{"buffered stream name [" + name + "] not found"};
  }
  if (print)
    medrctlog::info("Waiting one data from stream: {}", it->first);
  if (!it->second.wait_data_once_func())
  {
    return false;
  }
  if (print)
    medrctlog::info("Finished waiting one data from stream: {}", it->first);
  return true;
}

void StreamMap::getDataFromAllBufferedStreams(DataStore& data_store)
{
  for (auto it = buffered_stream_name_to_functions.begin();
       it != buffered_stream_name_to_functions.end();
       it++)
  {
    it->second.get_data_func(data_store);
  }
}

Task::Task()
{
}
Task::~Task()
{
}

bool IsValidPreviousState(
    const controller_state_t destination_state,
    const controller_state_t current_state)
{
  const auto& valid_prev_states =
      PREVIOUS_VALID_CONTROLLER_STATES.at(destination_state);
  if (std::find(
          valid_prev_states.begin(), valid_prev_states.end(), current_state) ==
      valid_prev_states.end())
  {
    return false;
  }
  return true;
}

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

ControllerInterface::ControllerInterface()
    : current_state(controller_state_t::UNINITIALIZED)
{
}
ControllerInterface::~ControllerInterface()
{
}
bool ControllerInterface::init(const std::string& controller_name)
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
bool ControllerInterface::enable()
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
bool ControllerInterface::disable()
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
bool ControllerInterface::clutch()
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
bool ControllerInterface::unclutch()
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
controller_state_t ControllerInterface::getState() const
{
  return current_state;
}
std::string ControllerInterface::getName() const
{
  return name;
}
bool ControllerInterface::onEnable()
{
  return true;
}
bool ControllerInterface::onDisable()
{
  return true;
}
bool ControllerInterface::onClutch()
{
  return true;
}
bool ControllerInterface::onUnclutch()
{
  return true;
}
void ControllerInterface::updateProcess(DataStore& input_data)
{
  input_stream_map.getDataFromAllBufferedStreams(input_data);
  update(input_data);
}
} // namespace controller
} // namespace medrct
