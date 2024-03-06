#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>

#include <medrct/stream/stream.hh>
#include <medrct/stream/stream_factory.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct/types/twist.hh>
#include <medrct/log.hh>
#include <medrct/controller/controller.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>

namespace medrct
{
namespace controller
{

class InputDeviceControl
{
public:
  InputDeviceControl(
      std::shared_ptr<stream::PubStream<Transform>> servo_cp_stream,
      std::shared_ptr<stream::PubStream<Wrench>> servo_cf_stream);
  void enable();
  void disable(const Transform& current_tf);
  void update();

private:
  Wrench empty_wrench;
  std::atomic<bool> is_enabled{false};
  std::shared_ptr<stream::PubStream<Transform>> servo_cp_stream;
  std::shared_ptr<stream::PubStream<Wrench>> servo_cf_stream;
};

struct CartesianTeleopControllerConfig
{
  std::string controller_name;
  std::shared_ptr<stream::SubStream<JointState>> measured_js_stream;
  std::shared_ptr<stream::PubStream<JointState>> output_js_stream;
  std::shared_ptr<env::ForwardKinematics> forward_kinematics;
  std::shared_ptr<env::InverseKinematics> inverse_kinematics;
  bool rotate_about_base_frame_vs_tip_frame = false;
  Quaternion output_2_output_ref_quat = Quaternion(1, 0, 0, 0);
  Quaternion input_2_input_ref_quat = Quaternion(1, 0, 0, 0);
  // TODO All types is aggragate for now
  // task_type_t task_type = task_type_t::AGGRAGATE;
  static void FromYaml(
      CartesianTeleopControllerConfig& ctcc,
      const YAML::Node controller_config,
      const stream::StreamFactory& stream_factory);
};

class CartesianTeleopController : public Controller
{
public:
  CartesianTeleopController();
  virtual ~CartesianTeleopController() = 0;
  bool init(const CartesianTeleopControllerConfig& config)
  {
    if (!config.measured_js_stream || !config.output_js_stream)
    {
      return false;
    }
    if (!config.forward_kinematics)
    {
      return false;
    }
    if (!config.inverse_kinematics)
    {
      return false;
    }
    measured_js_stream_name = config.measured_js_stream->name;
    this->forward_kinematics = config.forward_kinematics;
    this->inverse_kinematics = config.inverse_kinematics;
    input_stream_map.addWithBuffer(config.measured_js_stream);
    this->output_js_stream = config.output_js_stream;
    this->rotate_about_tip_frame_vs_base_frame =
        !config.rotate_about_base_frame_vs_tip_frame;

    h2m_rot = config.input_2_input_ref_quat.inverse().toRotationMatrix();
    s2e_rot = config.output_2_output_ref_quat.toRotationMatrix();

    return true;
  }

protected:
  Transform calculateOutputTfAndPublishJs(
      const Transform& input_diff_tf,
      const Transform& output_tf,
      const JointState& current_output_js);
  virtual bool onEnable() override;
  virtual bool onUnclutch() override;
  std::string measured_js_stream_name;
  std::shared_ptr<env::ForwardKinematics> forward_kinematics;
  std::shared_ptr<env::InverseKinematics> inverse_kinematics;
  Transform initial_output_tf;
  JointState command_output_js;
  std::shared_ptr<stream::PubStream<JointState>> output_js_stream;
  bool getInitialOutputTf();

private:
  bool rotate_about_tip_frame_vs_base_frame;
  Rotation h2m_rot;
  Rotation s2e_rot;
};

struct CartesianFollowerControllerConfig : CartesianTeleopControllerConfig
{
  std::shared_ptr<stream::SubStream<Transform>> input_callback_stream;
  real_t position_scale = 1.0;
  // Need both below for input device control
  std::shared_ptr<stream::PubStream<Transform>> servo_cp_stream;
  std::shared_ptr<stream::PubStream<Wrench>> servo_cf_stream;
  static void FromYaml(
      CartesianFollowerControllerConfig& cfcc,
      const YAML::Node controller_config,
      const stream::StreamFactory& stream_factory);
};

class CartesianFollowerController : public CartesianTeleopController
{
public:
  CartesianFollowerController();
  virtual ~CartesianFollowerController();
  bool init(const CartesianFollowerControllerConfig& config);

protected:
  real_t position_scale;
  std::string input_stream_name;
  Transform initial_input_tf;
  std::unique_ptr<InputDeviceControl> input_device_control;
  virtual bool onEnable() override;
  virtual bool onDisable() override;
  virtual bool onUnclutch() override;
  void update(const DataStore& input_data) override;
  bool getInitialInputTf();
};

struct CartesianIncrementControllerConfig : CartesianTeleopControllerConfig
{
  std::shared_ptr<stream::SubStream<Twist>> input_callback_stream;
  static void FromYaml(
      CartesianIncrementControllerConfig& cicc,
      const YAML::Node controller_config,
      const stream::StreamFactory& stream_factory);
};

class CartesianIncrementController : public CartesianTeleopController
{
public:
  CartesianIncrementController();
  virtual ~CartesianIncrementController();
  bool init(const CartesianIncrementControllerConfig& config);

protected:
  std::string input_stream_name;
  Transform current_command_output_tf;
  virtual bool onEnable() override;
  void update(const DataStore& input_data) override;
};
} // namespace controller
} // namespace medrct
