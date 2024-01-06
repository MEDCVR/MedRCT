#include <medrct_default_controller/cartesian_teleop_controller.hh>

#include <medrct/log.hh>

namespace medrct
{
namespace controller
{

using namespace medrct::stream;

CartesianTeleopController::CartesianTeleopController()
{
}
CartesianTeleopController::~CartesianTeleopController()
{
}

Transform CartesianTeleopController::calculateOutputTfAndPublishJs(
    const Transform& input_diff_tf, const Transform& output_tf)
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
      inverse_kinematics->computeIK(new_output_tf);

  // TODO, put joint names here;
  command_output_js.positions = ik_solutions[0];
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
  auto measured_stream_ptr =
      input_stream_map.get<SubStream<JointState>>(measured_js_stream_name);
  JointState current_output_js = measured_stream_ptr->getBuffer().getLatest();
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
  if (!CartesianTeleopController::init(config))
    return false;
  return initAggragate<Transform>(
      config.controller_name, config.input_callback_stream);
}

bool CartesianFollowerController::getInitialInputTf()
{
  if (!input_stream_map.waitForOneBufferedDataInput(input_stream_name, true))
    return false;
  auto input_stream_ptr =
      input_stream_map.get<SubStream<Transform>>(input_stream_name);
  initial_input_tf = input_stream_ptr->getBuffer().getLatest();
  return true;
}

bool CartesianFollowerController::onEnable()
{
  if (!CartesianTeleopController::onEnable())
    return false;
  return getInitialInputTf();
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
  Transform absolute_input_diff;
  absolute_input_diff.linear() =
      initial_input_tf.linear().inverse() * absolute_input_tf.linear();
  absolute_input_diff.translation() =
      (absolute_input_tf.translation() - initial_input_tf.translation()) *
      position_scale;

  this->calculateOutputTfAndPublishJs(absolute_input_diff, initial_output_tf);
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
  input_stream_name = config.input_callback_stream->name;
  input_stream_map.addWithBuffer(config.input_callback_stream);
  if (!CartesianTeleopController::init(config))
    return false;
  return initAggragate<Twist>(
      config.controller_name, config.input_callback_stream);
}

bool CartesianIncrementController::onEnable()
{
  if (!CartesianTeleopController::onEnable())
    return false;
  current_command_output_tf = initial_output_tf;
  return true;
}

void CartesianIncrementController::update(const DataStore& input_data)
{
  Twist input_twist = input_data.get<Twist>(input_stream_name);

  Transform input_diff_tf;
  input_diff_tf.translation() = input_twist.linear;
  input_diff_tf.linear() = FromRPY(input_twist.angular).toRotationMatrix();

  // calculate input diff tf;
  current_command_output_tf = this->calculateOutputTfAndPublishJs(
      input_diff_tf, current_command_output_tf);
  return;
}

} // namespace controller
} // namespace medrct
