#pragma once

#include <memory>
#include <string>

#include <medrct/stream/stream.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/controller/controller.hh>

namespace medrct
{
namespace controller
{
struct JointTeleopControllerConfig
{
  std::string controller_name;
  std::shared_ptr<stream::InputStream<JointState>> input_js_stream;
  std::shared_ptr<stream::InputStream<JointState>> measured_js_stream;
  std::shared_ptr<stream::OutputStream<JointState>> output_js_stream;
  // TODO All types is aggragate for now
  // task_type_t task_type = task_type_t::AGGRAGATE;
};

class JointTeleopController : public Controller
{
public:
  JointTeleopController();
  virtual ~JointTeleopController() = 0;
  bool init(const JointTeleopControllerConfig& init_config);

protected:
  std::string input_js_stream_name;
  std::string measured_js_stream_name;
  std::string output_js_stream_name;
};

class JointMimicController : public JointTeleopController
{
public:
  JointMimicController();
  virtual ~JointMimicController();
  bool init(const JointTeleopControllerConfig& init_config);

private:
  bool onEnable() override;
  void update(const DataStore& input_data) final;
  void matchJoints(
      const JointState& current_output_js, const JointState& current_input_js);
};

class JointIncrementController : public JointTeleopController
{
public:
  JointIncrementController();
  virtual ~JointIncrementController();
  bool init(const JointTeleopControllerConfig& init_config);

private:
  bool onEnable() override;
  void update(const DataStore& input_data) final;
  JointState current_command_output_js;
};
} // namespace controller
} // namespace medrct
