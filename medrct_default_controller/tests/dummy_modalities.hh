#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <string>

#include <medrct/intra_stream/intra_stream.hh>
#include <medrct/types/joint_state.hh>

using namespace medrct;
using namespace medrct::stream;

template <class T>
class InputModality
{
public:
  const std::string command_topic;
  InputModality(const std::string& command_topic) : command_topic(command_topic)
  {
    command_joint_stream = std::make_shared<IntraPubStream<T>>(
        command_topic, command_topic + "_output_stream");
  }
  void publish(const T& data) { command_joint_stream->publish(data); }

private:
  std::shared_ptr<IntraPubStream<T>> command_joint_stream;
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
    robot_command_js_stream = std::make_shared<IntraSubStream<JointState>>(
        command_js_topic, command_js_topic + "_input_stream");
    robot_command_js_stream->addCallback(
        "robotCommandCallback", robotCommandCallback);
    pe.registerProcess(robot_command_js_stream);
    robot_measured_js_stream = std::make_shared<IntraPubStream<JointState>>(
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
  std::shared_ptr<IntraPubStream<JointState>> robot_measured_js_stream;
  std::shared_ptr<IntraSubStream<JointState>> robot_command_js_stream;
  std::thread measured_js_thread;
  std::atomic<bool> is_running = {true};
};
