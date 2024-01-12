#pragma once

#include <functional>
#include <unordered_map>
#include <string>

#include "utils.hh"
#include <medrct/stream/stream.hh>

namespace medrct
{
namespace controller
{

enum class controller_state_t
{
  UNINITIALIZED,
  DISABLED,
  ENABLED,
  CLUTCHED,
  IN_ERROR
};

const static std::unordered_map<controller_state_t, std::string>
    CONTROLLER_STATE_TO_STRING = {
        {controller_state_t::UNINITIALIZED, "UNINITIALIZED"},
        {controller_state_t::DISABLED, "DISABLED"},
        {controller_state_t::ENABLED, "ENABLED"},
        {controller_state_t::CLUTCHED, "CLUTCHED"},
        {controller_state_t::IN_ERROR, "IN_ERROR"}};
const static std::
    unordered_map<controller_state_t, std::vector<controller_state_t>>
        PREVIOUS_VALID_CONTROLLER_STATES = {
            {controller_state_t::UNINITIALIZED, {}},
            {controller_state_t::DISABLED,
             {controller_state_t::UNINITIALIZED, controller_state_t::ENABLED}},
            {controller_state_t::ENABLED,
             {controller_state_t::DISABLED, controller_state_t::CLUTCHED}},
            {controller_state_t::CLUTCHED, {controller_state_t::ENABLED}},
            {controller_state_t::IN_ERROR,
             {controller_state_t::UNINITIALIZED,
              controller_state_t::DISABLED,
              controller_state_t::ENABLED,
              controller_state_t::CLUTCHED}}};

bool IsValidPreviousState(
    const controller_state_t destination_state,
    const controller_state_t current_state);

// A controller is able to enable/disable(), clutch/unclutch(). It has
// interfaces to read/write data
class Controller
{
public:
  using Ptr = std::shared_ptr<Controller>;
  using ConstPtr = std::shared_ptr<const Controller>;
  Controller();
  virtual ~Controller();
  bool enable();
  bool disable();
  bool clutch();
  bool unclutch();
  controller_state_t getState() const;
  std::string getName() const;

protected:
  virtual bool onEnable();
  virtual bool onDisable();
  virtual bool onClutch();
  virtual bool onUnclutch();
  virtual void update(const DataStore& input_data) = 0;

  bool initPeriodic(double rate_hz)
  {
    (void)rate_hz;
    throw std::logic_error{"Not yet implemented"};
    return false;
  }
  template <typename dataT>
  bool initAggragate(
      const std::string& controller_name,
      const std::shared_ptr<stream::SubStream<dataT>> input_callback_stream)
  {
    if (!init(controller_name))
    {
      return false;
    }
    // auto input_callback_stream =
    // std::dynamic_pointer_cast<SubStream<dataT>>(input_callback_stream);
    // if(!input_callback_stream)
    // {
    //   throw std::logic_error{"aggragate_stream must be of type
    //   SubStream"}; return false;
    // }
    task = std::make_unique<AggragateTask<dataT>>(
        input_stream_map,
        input_callback_stream,
        std::bind(&Controller::updateProcess, this, std::placeholders::_1));
    current_state = controller_state_t::DISABLED;
    return true;
  }

  template <typename T>
  const T getLatestFromBufferedInputStream(const std::string& stream_name) const
  {
    auto stream_ptr = input_stream_map.get<stream::SubStream<T>>(stream_name);
    return stream_ptr->getBuffer().getLatest();
  }
  StreamMap input_stream_map;

private:
  bool init(const std::string& controller_name);
  std::string name;
  void updateProcess(DataStore& input_data);
  std::unique_ptr<Task> task;
  controller_state_t current_state;
};
} // namespace controller
} // namespace medrct
