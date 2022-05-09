#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include <medrct_common/types.hh>
#include <medrct_common/joint_state.hh>
#include <medrct_common/intra/intra.hh>
#include <medrct_controller/controllers/joint_teleop_controller.hh>

#include "dummy_modalities.hh"

using namespace medrct;
using namespace medrct::stream;
using namespace medrct::controller;

class JointTeleopControllerTester
{
public:
  std::unique_ptr<SingleJointRobot> dummy_output;
  std::unique_ptr<InputModality<JointState>> input_modality;
  JointTeleopControllerTester(
      PipelineExecutor& pe,
      const std::vector<real_t>& input_js_positions,
      real_t start_joint_position = 0.0)
      : input_js_positions(input_js_positions)
  {
    dummy_output = std::make_unique<SingleJointRobot>(
        pe,
        std::bind(
            &JointTeleopControllerTester::robotCommandCallback,
            this,
            std::placeholders::_1),
        start_joint_position);
    input_modality = std::make_unique<InputModality<JointState>>("command_js");
  }
  ~JointTeleopControllerTester() {}

  JointTeleopControllerConfig init(PipelineExecutor& pe)
  {
    auto shared_input_js_stream =
        std::make_shared<SharedInputStream<JointState>>(
            input_modality->command_topic,
            input_modality->command_topic + "_input_stream");
    auto shared_measured_js_stream =
        std::make_shared<SharedInputStream<JointState>>(
            dummy_output->measured_js_topic,
            dummy_output->measured_js_topic + "_input_stream");
    pe.registerProcess(shared_input_js_stream);
    pe.registerProcess(shared_measured_js_stream);

    JointTeleopControllerConfig jc_cfg;
    jc_cfg.controller_name = "input-output_controller";
    jc_cfg.input_js_stream = shared_input_js_stream;
    jc_cfg.measured_js_stream = shared_measured_js_stream;
    jc_cfg.output_js_stream = std::make_shared<SharedOutputStream<JointState>>(
        dummy_output->command_js_topic,
        dummy_output->command_js_topic + "_output_stream");
    return jc_cfg;
  }

  void enableController(std::shared_ptr<ControllerInterface> ctrl)
  {
    std::thread thd([this, ctrl]() {
      // One or more streams need to pass the enable here, so keep publishing
      ctrl->enable(); // Waits here for the publisher
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    JointState input_js;
    input_js.push_back("dummy_joint_1", 0.0);
    for (int i = 0; i < 5; i++)
    {
      input_modality->publish(input_js);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    thd.join();
    is_enabled = true;
  }

  void publishCommandJses()
  {
    JointState input_js;
    input_js.push_back("dummy_joint_1", 0.0);
    for (auto& input_js_pos : input_js_positions)
    {
      input_js.positions[0] = input_js_pos;
      input_modality->publish(input_js);
    }
    return;
  }

private:
  void robotCommandCallback(const JointState& input_js)
  {
    // ASSERT_EQ(input_js.name.size(), 1);
    // ASSERT_EQ(input_js.name[0], joint_name);
    if (!is_enabled)
    { // Waits until the enable is finished
      return;
    }
    ASSERT_EQ(input_js.positions.size(), 1);
    return_js_positions.push_back(input_js.positions[0]);
    count++;
    if (count >= input_js_positions.size())
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
  std::vector<real_t> input_js_positions;
  std::vector<real_t> return_js_positions;

public:
  const std::string joint_name = "joint_1";
};

inline void CheckMimicResults(
    const std::vector<real_t>& input_js_positions,
    const std::vector<real_t>& return_js_positions)
{
  ASSERT_EQ(return_js_positions.size(), input_js_positions.size());
  for (unsigned int i = 0; i < input_js_positions.size(); ++i)
  {
    EXPECT_EQ(return_js_positions[i], input_js_positions[i]);
  }
  return;
}

TEST(TestMimicJointTeleopController, testOutput)
{
  StreamMaster::init();
  PipelineExecutor pe;

  // Initialize tester and controller
  std::vector<real_t> input_js_positions = {1, 3, 2, 4, 6, 5, 7, 9, 8};
  JointTeleopControllerTester jcontrol_tester(pe, input_js_positions);
  auto jc_cfg = jcontrol_tester.init(pe);

  auto jmc = std::make_shared<JointMimicController>();
  ASSERT_TRUE(jmc->init(jc_cfg));

  // Enable the controller
  jcontrol_tester.enableController(jmc);

  // Run the controller
  std::thread thd([&jcontrol_tester]() {
    std::unique_lock<std::mutex> lk(jcontrol_tester.mtx);
    jcontrol_tester.cv.wait_until(
        lk,
        std::chrono::system_clock::now() +
            std::chrono::seconds(10)); // 10s timeouts
  });
  jcontrol_tester.publishCommandJses();
  thd.join();

  // Check results
  CheckMimicResults(
      jcontrol_tester.input_js_positions, jcontrol_tester.return_js_positions);
}

inline void CheckIncrementResults(
    real_t start_js_position,
    const std::vector<real_t>& input_js_positions,
    const std::vector<real_t>& return_js_positions)
{
  real_t inc = start_js_position;
  ASSERT_EQ(return_js_positions.size(), input_js_positions.size());
  for (unsigned int i = 0; i < input_js_positions.size(); ++i)
  {
    inc += input_js_positions[i];
    EXPECT_EQ(return_js_positions[i], inc);
  }
  return;
}

TEST(TestIncrementJointTeleopController, testOutput)
{
  StreamMaster::init();
  PipelineExecutor pe;

  // Initialize tester and controller
  std::vector<real_t> input_js_positions = {1, 1, 1, 1, 1};
  real_t start_joint_position = 0.5;
  JointTeleopControllerTester jcontrol_tester(
      pe, input_js_positions, start_joint_position);

  auto jc_cfg = jcontrol_tester.init(pe);

  auto jic = std::make_shared<JointIncrementController>();
  ASSERT_TRUE(jic->init(jc_cfg));

  // Enable the controller
  jcontrol_tester.enableController(jic);

  // Run the controller
  std::thread thd([&jcontrol_tester]() {
    std::unique_lock<std::mutex> lk(jcontrol_tester.mtx);
    jcontrol_tester.cv.wait_until(
        lk,
        std::chrono::system_clock::now() +
            std::chrono::seconds(10)); // 10s timeouts
  });
  jcontrol_tester.publishCommandJses();
  thd.join();

  // Check results
  CheckIncrementResults(
      start_joint_position,
      jcontrol_tester.input_js_positions,
      jcontrol_tester.return_js_positions);
}
