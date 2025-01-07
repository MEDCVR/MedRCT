#include <climits>
#include <cstdlib>
#include <medrct/log.hh>
#include <variant>

#include <medrct/types/types.hh>
#include <medrct_default_controller/cartesian_teleop_controller.hh>
#include <medrct_default_controller/config.hh>

namespace medrct
{
namespace controller
{

using namespace medrct::stream;

void printQuaternion(Quaternion q)
{
  medrctlog::info(
      "x: {}\n, y: {}\n, z: {}\n, w: {}", q.x(), q.y(), q.z(), q.w());
}

void CartesianTeleopControllerConfig::FromYaml(
    CartesianTeleopControllerConfig& ctcc,
    const YAML::Node controller_config,
    const stream::StreamFactory& stream_factory)
{
  ctcc.controller_name = GetValue<std::string>(controller_config, "name");
  CreateOutputAndMeasuredStreams(
      ctcc.output_js_stream,
      ctcc.measured_js_stream,
      controller_config,
      stream_factory);
  CreateKinematicsSolvers(
      ctcc.forward_kinematics, ctcc.inverse_kinematics, controller_config);

  auto output_config = GetYamlNode(controller_config, "output");
  auto input_config = GetYamlNode(controller_config, "input");
  ctcc.rotate_about_base_frame_vs_tip_frame = GetValueDefault<bool>(
      output_config,
      "rotate_about_base_frame_vs_tip_frame",
      ctcc.rotate_about_base_frame_vs_tip_frame);
  ctcc.output_2_output_ref_quat = GetYamlQuaternionDefault(
      output_config, "to_reference_rotation", ctcc.output_2_output_ref_quat);
  ctcc.input_2_input_ref_quat = GetYamlQuaternionDefault(
      input_config, "to_reference_rotation", ctcc.input_2_input_ref_quat);
  ctcc.harmonize_joint_position_outputs = GetValueDefault<bool>(
      controller_config,
      "harmonize_joint_position_outputs",
      ctcc.harmonize_joint_position_outputs);
}

void CartesianFollowerControllerConfig::FromYaml(
    CartesianFollowerControllerConfig& cfcc,
    const YAML::Node controller_config,
    const stream::StreamFactory& stream_factory)
{
  CartesianTeleopControllerConfig::FromYaml(
      cfcc, controller_config, stream_factory);

  auto input_config = GetYamlNode(controller_config, "input");
  {
    YAML::Node n;
    n["topic_name"] = GetValue<std::string>(input_config, "topic");
    n["type"] = "input";
    n["name"] = cfcc.controller_name + "_input_stream";
    n["data_type"] =
        GetValueDefault<std::string>(input_config, "data_type", "Transform");
    cfcc.input_callback_stream =
        stream_factory.create<stream::SubStream<Transform>>(n);
  }
  cfcc.position_scale =
      GetValueDefault(input_config, "position_scale", cfcc.position_scale);

  if (input_config["hold_home_off"])
  {
    auto hold_home_off_config = GetYamlNode(input_config, "hold_home_off");
    YAML::Node n;
    n["topic_name"] =
        GetValue<std::string>(hold_home_off_config, "servo_cp_topic");
    n["type"] = "output";
    n["name"] = cfcc.controller_name + "_servo_cp_stream";
    n["data_type"] =
        GetValueDefault<std::string>(input_config, "data_type", "Transform");
    cfcc.servo_cp_stream =
        stream_factory.create<stream::PubStream<Transform>>(n);
    n["topic_name"] =
        GetValue<std::string>(hold_home_off_config, "servo_cf_topic");
    n["type"] = "output";
    n["name"] = cfcc.controller_name + "_servo_cf_stream";
    n["data_type"] = "Wrench";
    cfcc.servo_cf_stream = stream_factory.create<stream::PubStream<Wrench>>(n);
  }
  return;
}

void CartesianIncrementControllerConfig::FromYaml(
    CartesianIncrementControllerConfig& cicc,
    const YAML::Node controller_config,
    const stream::StreamFactory& stream_factory)
{
  CartesianTeleopControllerConfig::FromYaml(
      cicc, controller_config, stream_factory);
  YAML::Node n;
  YAML::Node input_config = GetYamlNode(controller_config, "input");
  n["topic_name"] = GetValue<std::string>(input_config, "topic");
  n["type"] = "input";
  n["name"] = cicc.controller_name + "_input_stream";
  std::string data_type =
      GetValueDefault<std::string>(input_config, "data_type", "Twist");
  n["data_type"] = data_type;

  if (data_type == "Twist")
  {
    cicc.input_callback_stream_variant =
        stream_factory.create<stream::SubStream<Twist>>(n);
  }
  else if (data_type == "Transform" || data_type == "Pose")
  {
    cicc.input_callback_stream_variant =
        stream_factory.create<stream::SubStream<Transform>>(n);
  }
  else
  {
    // TODO throw error
    return;
  }
  return;
}

InputDeviceControl::InputDeviceControl(
    std::shared_ptr<stream::PubStream<Transform>> servo_cp_stream,
    std::shared_ptr<stream::PubStream<Wrench>> servo_cf_stream)
    : servo_cp_stream(servo_cp_stream),
      servo_cf_stream(servo_cf_stream),
      empty_wrench(Wrench())
{
}
void InputDeviceControl::enable()
{
  is_enabled.store(true);
}
void InputDeviceControl::disable(const Transform& current_tf)
{
  servo_cp_stream->publish(current_tf);
  is_enabled.store(false);
}
void InputDeviceControl::update()
{
  if (is_enabled)
    servo_cf_stream->publish(empty_wrench);
}

CartesianTeleopController::CartesianTeleopController()
{
}
CartesianTeleopController::~CartesianTeleopController()
{
}

real_t sgn(real_t val)
{
  return (real_t(0) < val) - (val < real_t(0));
}

void GetHarmonizedJointPositions(
    JointState& to_harmonize, const JointState& from)
{
  for (size_t i = 0; i < to_harmonize.positions.size(); ++i)
  {
    real_t& to_pos = to_harmonize.positions[i];
    const real_t& from_pos = from.positions[i];
    real_t diff = std::abs(from_pos - to_pos);
    if (diff < 0.3)
      continue;
    real_t sign = sgn(from_pos - to_pos);
    real_t new_to_pos = to_pos + 2 * M_PI * sign;
    if (std::abs(from_pos - new_to_pos) < diff)
    {
      to_pos = new_to_pos;
    }
  }
  return;
}

Transform CartesianTeleopController::calculateOutputTfAndPublishJs(
    const Transform& input_diff_tf,
    const Transform& output_tf,
    const JointState& current_output_js)
{
  Vector3 input_diff_p_wrt_m = input_diff_tf.translation();
  Vector3 input_diff_p_wrt_h = h2m_rot * input_diff_p_wrt_m;
  Vector3 output_diff_p_wrt_e = input_diff_p_wrt_h;
  Vector3 output_diff_p_wrt_s = s2e_rot * output_diff_p_wrt_e;
  Vector3 output_p_wrt_s = output_tf.translation() + output_diff_p_wrt_s;

  Transform new_output_tf;
  new_output_tf.translation() = output_p_wrt_s;
  if (rotate_about_tip_frame_vs_base_frame)
    new_output_tf.linear() = output_tf.linear() * input_diff_tf.linear();
  else
    new_output_tf.linear() = input_diff_tf.linear() * output_tf.linear();

  std::vector<env::IKSolution> ik_solutions =
      inverse_kinematics->computeIK(new_output_tf, current_output_js.positions);

  if (ik_solutions.size() < 1)
  {
    medrctlog::error("Error! ik_solutions.size() < 1");
    return new_output_tf;
  }
  // TODO, put joint names here;
  command_output_js.positions = ik_solutions[0];
  if (command_output_js.positions.size() != current_output_js.positions.size())
  {
    medrctlog::error("Error! command_output_js.positions.size() != "
                     "current_output_js.positions.size(), could be something "
                     "wrong with computeIK");
    return new_output_tf;
  }
  // medrctlog::info("-----------------------------------------------");
  // medrctlog::info("command_output_js before:\n{}", command_output_js);
  if (harmonize_joint_position_outputs)
    GetHarmonizedJointPositions(command_output_js, current_output_js);
  // medrctlog::info("command_output_js:\n{}", command_output_js);
  // medrctlog::info("current_output_js:\n{}", current_output_js);
  output_js_stream->publish(command_output_js);
  return new_output_tf;
}

bool CartesianTeleopController::getInitialOutputTf()
{
  if (!input_stream_map.waitForOneBufferedDataInput(
          measured_js_stream_name, true))
  {
    return false;
  }
  JointState current_output_js =
      getLatestFromBufferedInputStream<JointState>(measured_js_stream_name);
  forward_kinematics->computeFK(initial_output_tf, current_output_js.positions);
  return true;
}

bool CartesianTeleopController::onEnable()
{
  return getInitialOutputTf();
}

bool CartesianTeleopController::onUnclutch()
{
  return getInitialOutputTf();
}

CartesianFollowerController::CartesianFollowerController()
{
}
CartesianFollowerController::~CartesianFollowerController()
{
}
bool CartesianFollowerController::init(
    const CartesianFollowerControllerConfig& config)
{
  input_stream_name = config.input_callback_stream->name;
  input_stream_map.addWithBuffer(config.input_callback_stream);
  position_scale = config.position_scale;
  if (config.servo_cf_stream && config.servo_cp_stream)
    input_device_control = std::make_unique<InputDeviceControl>(
        config.servo_cp_stream, config.servo_cf_stream);

  if (input_device_control)
  {
    medrctlog::info("Input device on");
  }

  if (!CartesianTeleopController::init(config))
    return false;
  return initAggragate<Transform>(
      config.controller_name, config.input_callback_stream);
}

bool CartesianFollowerController::getInitialInputTf()
{
  if (!input_stream_map.waitForOneBufferedDataInput(input_stream_name, true))
    return false;
  initial_input_tf =
      getLatestFromBufferedInputStream<Transform>(input_stream_name);
  return true;
}

bool CartesianFollowerController::onEnable()
{
  if (!CartesianTeleopController::onEnable())
    return false;
  if (input_device_control)
    input_device_control->enable();
  return getInitialInputTf();
}

bool CartesianFollowerController::onDisable()
{
  if (input_device_control)
  {
    auto current_tf =
        getLatestFromBufferedInputStream<Transform>(input_stream_name);
    input_device_control->disable(current_tf);
  }
  return true;
}

bool CartesianFollowerController::onUnclutch()
{
  if (!CartesianTeleopController::onUnclutch())
    return false;
  return getInitialInputTf();
}

void CartesianFollowerController::update(const DataStore& input_data)
{
  Transform absolute_input_tf = input_data.get<Transform>(input_stream_name);

  // medrctlog::info("absolute_input_tf");
  // medrctlog::info(absolute_input_tf.translation());
  // medrctlog::info(absolute_input_tf.linear());

  // medrctlog::info("initial_input_tf");
  // medrctlog::info(initial_input_tf.translation());
  // medrctlog::info(initial_input_tf.linear());

  Transform absolute_input_diff;
  absolute_input_diff.linear() =
      initial_input_tf.linear().inverse() * absolute_input_tf.linear();
  absolute_input_diff.translation() =
      (absolute_input_tf.translation() - initial_input_tf.translation()) *
      position_scale;
  // TODO; why this doesn't work?
  // auto js = input_data.get<JointState>(measured_js_stream_name);
  JointState current_output_js =
      input_data.get<JointState>(measured_js_stream_name);
  this->calculateOutputTfAndPublishJs(
      absolute_input_diff, initial_output_tf, current_output_js);
  if (input_device_control)
    input_device_control->update();
  return;
}

CartesianIncrementController::CartesianIncrementController()
{
}
CartesianIncrementController::~CartesianIncrementController()
{
}
bool CartesianIncrementController::init(
    const CartesianIncrementControllerConfig& config)
{
  if (std::holds_alternative<std::shared_ptr<stream::SubStream<Twist>>>(
          config.input_callback_stream_variant))
  {
    auto input_callback_stream =
        std::get<std::shared_ptr<stream::SubStream<Twist>>>(
            config.input_callback_stream_variant);
    input_stream_name = input_callback_stream->name;
    input_stream_map.addWithBuffer(input_callback_stream);
    if (!CartesianTeleopController::init(config))
      return false;
    input_diff_tf_process_func_ = std::bind(
        &CartesianIncrementController::processTwist,
        this,
        std::placeholders::_1);
    return initAggragate<Twist>(config.controller_name, input_callback_stream);
  }
  else if (std::holds_alternative<
               std::shared_ptr<stream::SubStream<Transform>>>(
               config.input_callback_stream_variant))
  {
    auto input_callback_stream =
        std::get<std::shared_ptr<stream::SubStream<Transform>>>(
            config.input_callback_stream_variant);
    input_stream_name = input_callback_stream->name;
    input_stream_map.addWithBuffer(input_callback_stream);
    if (!CartesianTeleopController::init(config))
      return false;
    input_diff_tf_process_func_ = std::bind(
        &CartesianIncrementController::processTransform,
        this,
        std::placeholders::_1);
    return initAggragate<Transform>(
        config.controller_name, input_callback_stream);
  }
  //
  // variant input not allowed
  return false;
}

bool CartesianIncrementController::onEnable()
{
  if (!CartesianTeleopController::onEnable())
    return false;
  current_command_output_tf = initial_output_tf;
  return true;
}

Transform
CartesianIncrementController::processTwist(const DataStore& input_data)
{
  Twist input_twist = input_data.get<Twist>(input_stream_name);

  Transform input_diff_tf;
  input_diff_tf.translation() = input_twist.linear;
  input_diff_tf.linear() = FromRPY(input_twist.angular).toRotationMatrix();
  return input_diff_tf;
}

Transform
CartesianIncrementController::processTransform(const DataStore& input_data)
{
  return input_data.get<Transform>(input_stream_name);
}

void CartesianIncrementController::update(const DataStore& input_data)
{
  Transform input_diff_tf = input_diff_tf_process_func_(input_data);
  // calculate input diff tf;
  JointState current_output_js =
      input_data.get<JointState>(measured_js_stream_name);
  current_command_output_tf = this->calculateOutputTfAndPublishJs(
      input_diff_tf, current_command_output_tf, current_output_js);
  return;
}

} // namespace controller
} // namespace medrct
