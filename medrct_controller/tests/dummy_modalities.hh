#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <string>

#include <medrct_common/intra/intra.hh>
#include <medrct_common/joint_state.hh>

using namespace medrct;
using namespace medrct::stream;

template <class T>
class InputModality
{
public:
  const std::string command_topic;
  InputModality(const std::string& command_topic) : command_topic(command_topic)
  {
    command_joint_stream = std::make_shared<SharedOutputStream<T>>(
        command_topic, command_topic + "_output_stream");
  }
  void publish(const T& data) { command_joint_stream->publish(data); }

private:
  std::shared_ptr<SharedOutputStream<T>> command_joint_stream;
};

class SingleJointRobot
{
public:
  const std::string measured_js_topic = "robot_measured_js";
  const std::string command_js_topic = "robot_command_js";

  SingleJointRobot(
      PipelineExecutor& pe,
      const std::function<void(const JointState&)>& robotCommandCallback,
      double start_joint_position = 0.0)
      : start_joint_position(start_joint_position)
  {
    robot_command_js_stream = std::make_shared<SharedInputStream<JointState>>(
        command_js_topic, command_js_topic + "_input_stream");
    robot_command_js_stream->addCallback(
        "robotCommandCallback", robotCommandCallback);
    pe.registerProcess(robot_command_js_stream);
    robot_measured_js_stream = std::make_shared<SharedOutputStream<JointState>>(
        measured_js_topic, measured_js_topic + "_output_stream");
    measured_js_thread = std::thread(
        std::bind(&SingleJointRobot::keepPublishRobotMeasuredJs, this));
  }

  ~SingleJointRobot()
  {
    is_running.store(false);
    measured_js_thread.join();
  }

  void keepPublishRobotMeasuredJs()
  {
    JointState measured_js;
    measured_js.push_back("joint_1", start_joint_position);
    while (is_running.load())
    {
      robot_measured_js_stream->publish(measured_js);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

private:
  const double start_joint_position;
  std::shared_ptr<SharedOutputStream<JointState>> robot_measured_js_stream;
  std::shared_ptr<SharedInputStream<JointState>> robot_command_js_stream;
  std::thread measured_js_thread;
  std::atomic<bool> is_running = {true};
};
