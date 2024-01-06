#pragma once

#include <memory>
#include <string>

#include <medrct/stream/stream.hh>
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
template <typename inputT>
struct CartesianTeleopControllerConfig
{
  std::string controller_name;
  std::shared_ptr<stream::SubStream<inputT>> input_callback_stream;
  std::shared_ptr<stream::SubStream<JointState>> measured_js_stream;
  std::shared_ptr<stream::PubStream<JointState>> output_js_stream;
  std::shared_ptr<env::ForwardKinematics> forward_kinematics;
  std::shared_ptr<env::InverseKinematics> inverse_kinematics;
  double position_scale = 1.0;
  bool rotate_about_tip_frame_vs_base_frame = false;
  Quaternion output_2_output_ref_quat = Quaternion(1, 0, 0, 0);
  Quaternion input_2_input_ref_quat = Quaternion(1, 0, 0, 0);
  // TODO All types is aggragate for now
  // task_type_t task_type = task_type_t::AGGRAGATE;
};

class CartesianTeleopController : public Controller
{
public:
  CartesianTeleopController();
  virtual ~CartesianTeleopController() = 0;
  template <class inputT>
  bool init(const CartesianTeleopControllerConfig<inputT>& config)
  {
    if (!config.input_callback_stream || !config.measured_js_stream ||
        !config.output_js_stream)
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
    input_stream_name = config.input_callback_stream->name;
    measured_js_stream_name = config.measured_js_stream->name;
    this->forward_kinematics = config.forward_kinematics;
    this->inverse_kinematics = config.inverse_kinematics;
    position_scale = config.position_scale;
    input_stream_map.addWithBuffer(config.input_callback_stream);
    input_stream_map.addWithBuffer(config.measured_js_stream);
    this->output_js_stream = config.output_js_stream;
    this->rotate_about_tip_frame_vs_base_frame =
        config.rotate_about_tip_frame_vs_base_frame;

    h2m_rot = config.input_2_input_ref_quat.inverse().toRotationMatrix();
    s2e_rot = config.output_2_output_ref_quat.toRotationMatrix();
    return initAggragate<inputT>(
        config.controller_name, config.input_callback_stream);
  }

protected:
  Transform calculateOutputTfAndPublishJs(
      const Transform& input_diff_tf, const Transform& output_tf);
  virtual bool onEnable() override;
  virtual bool onUnclutch() override;
  std::string input_stream_name;
  std::string measured_js_stream_name;
  std::shared_ptr<env::ForwardKinematics> forward_kinematics;
  std::shared_ptr<env::InverseKinematics> inverse_kinematics;
  Transform initial_output_tf;
  JointState command_output_js;
  std::shared_ptr<stream::PubStream<JointState>> output_js_stream;

private:
  bool getInitialOutputTf();
  bool rotate_about_tip_frame_vs_base_frame;
  double position_scale;
  Rotation h2m_rot;
  Rotation s2e_rot;
};

typedef CartesianTeleopControllerConfig<Transform>
    CartesianFollowerControllerConfig;
class CartesianFollowerController : public CartesianTeleopController
{
public:
  CartesianFollowerController();
  virtual ~CartesianFollowerController();
  bool init(const CartesianFollowerControllerConfig& init_config);

private:
  Transform initial_input_tf;
  bool getInitialInputTf();
  virtual bool onEnable() final;
  virtual bool onUnclutch() final;
  void update(const DataStore& input_data) final;
};

typedef CartesianTeleopControllerConfig<Twist>
    CartesianIncrementControllerConfig;
class CartesianIncrementController : public CartesianTeleopController
{
public:
  CartesianIncrementController();
  virtual ~CartesianIncrementController();
  bool init(const CartesianIncrementControllerConfig& init_config);

private:
  Transform current_command_output_tf;
  virtual bool onEnable() final;
  void update(const DataStore& input_data) final;
};
} // namespace controller
} // namespace medrct
