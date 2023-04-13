#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <medrct/log.hh>
#include <medrct/types/types.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/types/twist.hh>
#include <medrct/intra_stream/intra_stream.hh>

#include <medrct_default_controller/cartesian_teleop_controller.hh>

#include "dummy_modalities.hh"
#include "dummy_kinematics.hh"

using namespace medrct;
using namespace medrct::stream;
using namespace medrct::controller;

template <class T>
class CartesianTeleopControllerTester
{
public:
  std::unique_ptr<SingleJointRobot> dummy_output;
  std::unique_ptr<InputModality<T>> input_modality;
  CartesianTeleopControllerTester(
      PipelineExecutor& pe,
      unsigned int input_size_to_finish,
      real_t output_start_joint_position = 0.0)
      : input_size_to_finish(input_size_to_finish)
  {
    dummy_output = std::make_unique<SingleJointRobot>(
        pe,
        std::bind(
            &CartesianTeleopControllerTester::robotCommandCallback,
            this,
            std::placeholders::_1),
        output_start_joint_position);
    input_modality = std::make_unique<InputModality<T>>("command_tf");
  }
  ~CartesianTeleopControllerTester() {}

  CartesianTeleopControllerConfig<T> init(PipelineExecutor& pe)
  {
    auto shared_input_stream = std::make_shared<IntraInputStream<T>>(
        input_modality->command_topic,
        input_modality->command_topic + "_input_stream");
    auto shared_measured_js_stream =
        std::make_shared<IntraInputStream<JointState>>(
            dummy_output->measured_js_topic,
            dummy_output->measured_js_topic + "_input_stream");
    pe.registerProcess(shared_input_stream);
    pe.registerProcess(shared_measured_js_stream);

    CartesianTeleopControllerConfig<T> cc_cfg;
    cc_cfg.controller_name = "input-output_controller";
    cc_cfg.input_callback_stream = shared_input_stream;
    cc_cfg.measured_js_stream = shared_measured_js_stream;
    cc_cfg.output_js_stream = std::make_shared<IntraOutputStream<JointState>>(
        dummy_output->command_js_topic,
        dummy_output->command_js_topic + "_output_stream");

    cc_cfg.forward_kinematics = std::make_shared<XPrismaticForwardKinematics>();
    cc_cfg.inverse_kinematics = std::make_shared<XPrismaticInverseKinematics>();
    return cc_cfg;
  }
  void enableController(std::shared_ptr<Controller> ctrl, T input)
  {
    std::thread thd([this, ctrl]() {
      // One or more streams need to pass the enable here, so keep publishing
      ctrl->enable(); // Waits here for the publisher
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (int i = 0; i < 5; i++)
    {
      input_modality->publish(input);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    thd.join();
    is_enabled = true;
  }

  void publishCommands(const std::vector<T>& input_commands)
  {
    for (const auto& input_command : input_commands)
    {
      input_modality->publish(input_command);
    }
    return;
  }

private:
  void robotCommandCallback(const JointState& data_js)
  {
    if (!is_enabled)
    { // Waits until the enable is finished
      return;
    }
    ASSERT_EQ(data_js.positions.size(), 1);
    return_js_positions.push_back(data_js.positions[0]);
    count++;
    if (count >= input_size_to_finish)
    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.notify_all();
    }
  };
  unsigned int count = 0;

public:
  bool is_enabled = false;
  std::mutex mtx;
  std::condition_variable cv;
  unsigned int input_size_to_finish;
  std::vector<real_t> return_js_positions;

public:
  const std::string joint_name = "joint_1";
};

TEST(TestCartesianFollowerController, testOutput)
{
  medrctlog::set_level(medrctlog::level::info); // Set global log level to
  StreamMaster::init();
  PipelineExecutor pe;

  // Initialize tester and controller
  real_t initial_input_x = 1;
  real_t initial_output_x =
      2; // for Xprismatic robot, output_x is same as output_joint
  std::vector<real_t> input_x_positions = {1, 3, 2, 4, 3, 2, 0, -1, -3, -1};
  std::vector<Transform> input_tfs;
  for (unsigned int i = 0; i < input_x_positions.size(); ++i)
  {
    Transform tf;
    tf.translation().x() = input_x_positions[i];
    input_tfs.push_back(tf);
  }

  CartesianTeleopControllerTester<Transform> ccontrol_tester(
      pe, input_x_positions.size(), initial_output_x);
  auto cc_cfg = ccontrol_tester.init(pe);

  auto cic = std::make_shared<CartesianFollowerController>();
  ASSERT_TRUE(cic->init(cc_cfg));

  // Enable the controller
  Transform initial_input_tf;
  initial_input_tf.translation().x() = initial_input_x;
  ccontrol_tester.enableController(cic, initial_input_tf);

  // Run the controller
  std::thread thd([&ccontrol_tester]() {
    std::unique_lock<std::mutex> lk(ccontrol_tester.mtx);
    ccontrol_tester.cv.wait_until(
        lk,
        std::chrono::system_clock::now() +
            std::chrono::seconds(10)); // 10s timeouts
  });
  ccontrol_tester.publishCommands(input_tfs);
  thd.join();

  // Check results
  ASSERT_EQ(
      ccontrol_tester.return_js_positions.size(), input_x_positions.size());
  for (unsigned int i = 0; i < input_x_positions.size(); ++i)
  {
    JointState js;
    js.push_back("", ccontrol_tester.return_js_positions[i]);

    Transform output_tf_result;
    cc_cfg.forward_kinematics->computeFK(output_tf_result, js.positions);
    medrctlog::debug("---index---: {}", i);
    medrctlog::debug("input_tf_x: {}", input_x_positions[i]);
    medrctlog::debug("output_tf_x: {}", output_tf_result.translation().x());

    auto out_diff_x = output_tf_result.translation().x() - initial_output_x;
    auto in_diff_x = input_x_positions[i] - initial_input_x;
    medrctlog::debug("input_diff_tf_x: {}", in_diff_x);
    medrctlog::debug("output_diff_tf_x: {}", out_diff_x);
    ASSERT_EQ(out_diff_x, in_diff_x);
  }
  return;
}

TEST(TestCartesianIncrementController, testOutput)
{
  medrctlog::set_level(medrctlog::level::info); // Set global log level to debug
  StreamMaster::init();
  PipelineExecutor pe;

  // Initialize tester and controller
  real_t initial_input_x = 1;
  real_t initial_output_x =
      2; // for Xprismatic robot, output_x is same as output_joint
  std::vector<real_t> input_x_positions = {1, 2, -2, -1, 1, 3, 3, -3};
  std::vector<Twist> input_twists;
  for (unsigned int i = 0; i < input_x_positions.size(); ++i)
  {
    Twist twist;
    twist.linear.x() = input_x_positions[i];
    input_twists.push_back(twist);
  }

  CartesianTeleopControllerTester<Twist> ccontrol_tester(
      pe, input_x_positions.size(), initial_output_x);
  auto cc_cfg = ccontrol_tester.init(pe);

  auto cfc = std::make_shared<CartesianIncrementController>();
  ASSERT_TRUE(cfc->init(cc_cfg));

  // Enable the controller
  Twist initial_twist;
  initial_twist.linear.x() = 0; // this has to be zero, or else internal
  // command tf in controller will get incrmented
  ccontrol_tester.enableController(cfc, initial_twist);

  // Run the controller
  std::thread thd([&ccontrol_tester]() {
    std::unique_lock<std::mutex> lk(ccontrol_tester.mtx);
    ccontrol_tester.cv.wait_until(
        lk,
        std::chrono::system_clock::now() +
            std::chrono::seconds(10)); // 10s timeouts
  });
  ccontrol_tester.publishCommands(input_twists);
  thd.join();

  // Check results
  ASSERT_EQ(
      ccontrol_tester.return_js_positions.size(), input_x_positions.size());
  real_t accumulated_x = initial_output_x;
  for (unsigned int i = 0; i < input_x_positions.size(); ++i)
  {
    JointState js;
    js.push_back("", ccontrol_tester.return_js_positions[i]);
    Transform output_tf_result;
    cc_cfg.forward_kinematics->computeFK(output_tf_result, js.positions);
    accumulated_x += input_x_positions[i];
    medrctlog::debug("---index---: {}", i);
    medrctlog::debug("js: {}", js.positions[i]);
    medrctlog::debug("input_x_positions: {}", input_x_positions[i]);
    medrctlog::debug("accumulated_x: {}", accumulated_x);
    medrctlog::debug("output_tf_x: {}", output_tf_result.translation().x());
    ASSERT_EQ(output_tf_result.translation().x(), accumulated_x);
  }
  return;
}
