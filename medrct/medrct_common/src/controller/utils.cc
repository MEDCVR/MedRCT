#include <algorithm>

#include <medrct/log.hh>
#include <medrct/controller/controller.hh>

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
} // namespace controller
} // namespace medrct
