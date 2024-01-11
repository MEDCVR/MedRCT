// uncomment to disable assert()
// #define NDEBUG
#include <assert.h>
#include <chrono>
#include <thread>

#include <medrct/log.hh>
#include <medrct_default_controller/config.hh>
#include <medrct_default_controller/joint_teleop_controller.hh>

namespace medrct
{
namespace controller
{

using namespace medrct::stream;

void JointTeleopControllerConfig::FromYaml(
    JointTeleopControllerConfig& jcc,
    const YAML::Node controller_config,
    const stream::StreamFactory& stream_factory)
{
  jcc.controller_name = GetValue<std::string>(controller_config, "name");
  YAML::Node n;
  n["topic_name"] =
      GetValue<std::string>(GetYamlNode(controller_config, "input"), "topic");
  n["type"] = "input";
  n["name"] = jcc.controller_name + "_input_stream";
  n["data_type"] = "JointState";
  jcc.input_js_stream = stream_factory.create<stream::SubStream<JointState>>(n);
  CreateOutputAndMeasuredStreams(
      jcc.output_js_stream,
      jcc.measured_js_stream,
      controller_config,
      stream_factory);
}

JointTeleopController::JointTeleopController()
{
}
JointTeleopController::~JointTeleopController()
{
}
bool JointTeleopController::init(const JointTeleopControllerConfig& init_config)
{
  if (!init_config.input_js_stream || !init_config.measured_js_stream ||
      !init_config.output_js_stream)
  {
    return false;
  }
  input_js_stream_name = init_config.input_js_stream->name;
  measured_js_stream_name = init_config.measured_js_stream->name;
  input_stream_map.addWithBuffer(init_config.input_js_stream);
  input_stream_map.addWithBuffer(init_config.measured_js_stream);
  this->output_js_stream = init_config.output_js_stream;
  return initAggragate(
      init_config.controller_name, init_config.input_js_stream);
}

JointMimicController::JointMimicController()
{
}
JointMimicController::~JointMimicController()
{
}
bool JointMimicController::init(const JointTeleopControllerConfig& init_config)
{
  return JointTeleopController::init(init_config);
}

bool JointMimicController::onEnable()
{
  if (!input_stream_map.waitForOneBufferedDataInput(
          measured_js_stream_name, true))
    return false;
  if (!input_stream_map.waitForOneBufferedDataInput(input_js_stream_name, true))
    return false;

  auto measured_stream_ptr =
      input_stream_map.get<SubStream<JointState>>(measured_js_stream_name);
  JointState current_output_js = measured_stream_ptr->getBuffer().getLatest();

  auto input_stream_ptr =
      input_stream_map.get<SubStream<JointState>>(input_js_stream_name);
  JointState current_input_js = input_stream_ptr->getBuffer().getLatest();

  // TODO maybe have a joint verify
  // if (current_output_js.names.size() != current_input_js.names.size())
  // {
  //   // Maybe need a false transition error here.
  //   medrctlog::error("names size not same");
  //   return false;
  // }
  if (current_output_js.positions.size() != current_input_js.positions.size())
  {
    // Maybe need a false transition error here.
    medrctlog::error("positions size not same");
    return false;
  }

  matchJoints(current_output_js, current_input_js);
  // Check if far from current mimiced position
  return true;
}

// The difference is other_js.positions[index] - this_js.positions[index]
bool IsAllJointsMatch(
    std::vector<std::pair<unsigned int, real_t>>&
        index_and_differences_to_match,
    const JointState& this_js,
    const JointState& other_js)
{
  real_t tolerance = 0.1; // radian
  index_and_differences_to_match.clear();
  for (unsigned int i = 0; i < other_js.positions.size(); ++i)
  {
    real_t other_this_diff = other_js.positions[i] - this_js.positions[i];
    if (std::abs(other_this_diff) > tolerance)
    {
      index_and_differences_to_match.push_back(
          std::pair<unsigned int, real_t>(i, other_this_diff));
    }
  }
  return index_and_differences_to_match.size() == 0;
}
inline real_t Sign(real_t val)
{
  return (0 < val) - (val < 0);
}

// TODO currently this is open loop, do we want to do closed loop and get from
// feedback js?
void JointMimicController::matchJoints(
    const JointState& current_output_js, const JointState& current_input_js)
{
  JointState current_command_js = current_output_js;
  real_t increment = 0.05; // rad == 2.86 deg
  real_t rate = 10;        // hz
  // speed is 2.86 * 10 = 28.6 deg/s
  int sleep_time = 1000 / rate; // ms
  std::vector<std::pair<unsigned int, real_t>> index_and_differences_to_match;
  while (!IsAllJointsMatch(
      index_and_differences_to_match, current_command_js, current_input_js))
  {
    for (auto index_diff_to_match : index_and_differences_to_match)
    {
      current_command_js.positions[index_diff_to_match.first] +=
          Sign(index_diff_to_match.second) * increment;
    }
    output_js_stream->publish(current_command_js);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
  }
}

void JointMimicController::update(const DataStore& input_data)
{
  // Should do some interpolation to set the similar joints in sync:
  JointState input_js = input_data.get<JointState>(input_js_stream_name);
  JointState output_js;
  output_js.positions.reserve(input_js.positions.size());
  for (const auto& pos : input_js.positions)
  {
    // TODO Scale here
    output_js.positions.push_back(pos);
  }
  output_js_stream->publish(output_js);
}

JointIncrementController::JointIncrementController()
{
}
JointIncrementController::~JointIncrementController()
{
}
bool JointIncrementController::init(
    const JointTeleopControllerConfig& init_config)
{
  return JointTeleopController::init(init_config);
}

void JointIncrementController::update(const DataStore& input_data)
{
  JointState input_js = input_data.get<JointState>(input_js_stream_name);
  if (input_js.positions.size() != current_command_output_js.positions.size())
  {
    medrctlog::error(
        "Joint positions from input is not same size as from {}.",
        measured_js_stream_name);
    return;
  }
  for (unsigned int i = 0; i < input_js.positions.size(); ++i)
  {
    current_command_output_js.positions[i] =
        input_js.positions[i] + current_command_output_js.positions[i];
  }
  output_js_stream->publish(current_command_output_js);
}

bool JointIncrementController::onEnable()
{
  if (!input_stream_map.waitForOneBufferedDataInput(
          measured_js_stream_name, true))
    return false;
  auto measured_stream_ptr =
      input_stream_map.get<SubStream<JointState>>(measured_js_stream_name);
  current_command_output_js = measured_stream_ptr->getBuffer().getLatest();
  return true;
}
} // namespace controller
} // namespace medrct
