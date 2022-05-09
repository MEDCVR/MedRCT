#include <medrct_controller/controllers/cartesian_teleop_controller.hh>

#include <medrct_common/log.hh>

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
  // TODO: do a rotation adjustment that makes sense
  Transform input_diff_rotated = input_diff_tf;

  Transform new_output_tf;
  new_output_tf.linear() = output_tf.linear() * input_diff_rotated.linear();
  new_output_tf.translation() =
      output_tf.translation() + input_diff_rotated.translation();

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
      input_stream_map.get<InputStream<JointState>>(measured_js_stream_name);
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
    const CartesianFollowerControllerConfig& init_config)
{
  return CartesianTeleopController::init<Transform>(init_config);
}

bool CartesianFollowerController::getInitialInputTf()
{
  if (!input_stream_map.waitForOneBufferedDataInput(input_stream_name, true))
    return false;
  auto input_stream_ptr =
      input_stream_map.get<InputStream<Transform>>(input_stream_name);
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
  Transform absolute_input_diff =
      initial_input_tf.inverse() * absolute_input_tf;
  absolute_input_diff.translation() =
      absolute_input_tf.translation() -
      initial_input_tf.translation() * position_scale;

  this->calculateOutputTfAndPublishJs(absolute_input_diff, initial_output_tf);
}

CartesianIncrementController::CartesianIncrementController()
{
}
CartesianIncrementController::~CartesianIncrementController()
{
}
bool CartesianIncrementController::init(
    const CartesianIncrementControllerConfig& init_config)
{
  return CartesianTeleopController::init<Twist>(init_config);
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
}

} // namespace controller
} // namespace medrct
