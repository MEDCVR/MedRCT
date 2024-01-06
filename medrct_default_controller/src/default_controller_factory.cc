#include <string>

#include <medrct/loader/class_loader.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/log.hh>
#include <medrct/config.hh>

#include <medrct_env/description/world.hh>
#include <medrct_env/kinematics/kinematics_factory.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>

#include <medrct_default_controller/default_controller_factory.hh>
#include <medrct_default_controller/cartesian_teleop_controller.hh>
#include <medrct_default_controller/joint_teleop_controller.hh>

namespace medrct
{
namespace controller
{

void CreateOutputAndMeasuredStreams(
    stream::PubStream<JointState>::Ptr& output_js_stream,
    stream::SubStream<JointState>::Ptr& measured_js_stream,
    const YAML::Node& controller_config,
    const stream::StreamFactory& stream_factory)
{
  YAML::Node output_config = GetYamlNode(controller_config, "output");
  std::string name = GetValue<std::string>(controller_config, "name");

  YAML::Node n;
  n["name"] = name + "_output_js_stream";
  n["type"] = "output";
  n["data_type"] = "JointState";
  n["topic_name"] = GetValue<std::string>(output_config, "control_topic");
  output_js_stream = stream_factory.create<stream::PubStream<JointState>>(n);

  n["name"] = name + "_feedback_js_stream";
  n["type"] = "input";
  n["data_type"] = "JointState";
  n["topic_name"] = GetValue<std::string>(output_config, "feedback_topic");
  measured_js_stream = stream_factory.create<stream::SubStream<JointState>>(n);
  return;
}

void CreateKinematicsSolvers(
    std::shared_ptr<env::ForwardKinematics>& forward_kinematics,
    std::shared_ptr<env::InverseKinematics>& inverse_kinematics,
    const YAML::Node& controller_config)
{
  YAML::Node kinematics_config = GetYamlNode(controller_config, "kinematics");
  medrct::env::KinematicsTree::Ptr kin_tree = nullptr;
  std::string kin_tree_name = GetValueDefault<std::string>(kinematics_config, "tree", "");
  if (kin_tree_name != "")
  {
    kin_tree = env::World::getInstance().getKinematicsTree(kin_tree_name);
    if (!kin_tree)
      throw std::runtime_error("Kinematics tree not found from URDF");
  }

  YAML::Node forward_kin_config = GetYamlNode(kinematics_config, "forward");
  env::ForwardKinematicsFactory::Ptr forward_kinematics_factory;
  try
  {
    forward_kinematics_factory =
        medrct_loader::createSharedInstance<env::ForwardKinematicsFactory>(
            forward_kin_config);
    forward_kinematics = forward_kinematics_factory->create(
        kin_tree, GetYamlNode(forward_kin_config, "config"));
  }
  catch (const std::exception& e)
  {
    throw std::invalid_argument(
        "Error in instantiating forward kinematics: " + std::string(e.what()));
  }

  YAML::Node inverse_kin_config = GetYamlNode(kinematics_config, "inverse");
  env::InverseKinematicsFactory::Ptr inverse_kinematics_factory;
  try
  {
    inverse_kinematics_factory =
        medrct_loader::createSharedInstance<env::InverseKinematicsFactory>(
            inverse_kin_config);
    inverse_kinematics = inverse_kinematics_factory->create(
        kin_tree, GetYamlNode(inverse_kin_config, "config"));
  }
  catch (const std::exception& e)
  {
    throw std::invalid_argument(
        "Error in inverse_kinematics config: " + std::string(e.what()));
  }
}

Controller::Ptr DefaultControllerFactory::create(
    const medrct::stream::StreamFactory& stream_factory,
    YAML::Node controller_config) const
{
  std::string class_name = GetValue<std::string>(GetYamlNode(controller_config, "loader"), "create_class_name");
  std::string controller_name = GetValue<std::string>(controller_config, "name");
  YAML::Node n;
  n["topic_name"] = GetValue<std::string>(GetYamlNode(controller_config, "input"), "topic");
  n["type"] = "input";

  if (class_name == "JointMimicController" ||
      class_name == "JointIncrementController")
  {
    JointTeleopControllerConfig jcc;
    jcc.controller_name = controller_name;
    n["name"] = jcc.controller_name + "_input_stream";
    n["data_type"] = "JointState";
    jcc.input_js_stream =
        stream_factory.create<stream::SubStream<JointState>>(n);
    CreateOutputAndMeasuredStreams(
        jcc.output_js_stream,
        jcc.measured_js_stream,
        controller_config,
        stream_factory);

    if (class_name == "JointMimicController")
    {
      auto jmc = std::make_shared<JointMimicController>();
      if (!jmc->init(jcc))
        throw std::invalid_argument(
            "Failed to init JointMimicController with name: " +
            jcc.controller_name);
    }
    else // Must be JointIncrementController
    {
      auto jic = std::make_shared<JointIncrementController>();
      if (!jic->init(jcc))
        throw std::invalid_argument(
            "Failed to init JointIncrementController with name: " +
            jcc.controller_name);
    }
  }
  else if (class_name == "CartesianIncrementController")
  {
    CartesianIncrementControllerConfig cic_cfg;
    cic_cfg.controller_name = controller_name;
    n["name"] = cic_cfg.controller_name + "_input_stream";
    n["data_type"] = "Twist";
    cic_cfg.input_callback_stream =
        stream_factory.create<stream::SubStream<Twist>>(n);
    CreateOutputAndMeasuredStreams(
        cic_cfg.output_js_stream,
        cic_cfg.measured_js_stream,
        controller_config,
        stream_factory);
    CreateKinematicsSolvers(
        cic_cfg.forward_kinematics,
        cic_cfg.inverse_kinematics,
        controller_config);
    Transform tf;
    cic_cfg.forward_kinematics->computeFK(tf, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto cic = std::make_shared<CartesianIncrementController>();
    if (!cic->init(cic_cfg))
      throw std::invalid_argument(
          "Failed to init CartesianIncrementController with name: " +
          cic_cfg.controller_name);
    return cic;
  }
  else if (class_name == "CartesianFollowerController")
  {
    CartesianFollowerControllerConfig cfc_cfg;
    cfc_cfg.controller_name = controller_name;
    n["name"] = cfc_cfg.controller_name + "_input_stream";
    n["data_type"] = "Transform";
    cfc_cfg.input_callback_stream =
        stream_factory.create<stream::SubStream<Transform>>(n);
    CreateOutputAndMeasuredStreams(
        cfc_cfg.output_js_stream,
        cfc_cfg.measured_js_stream,
        controller_config,
        stream_factory);
    CreateKinematicsSolvers(
        cfc_cfg.forward_kinematics,
        cfc_cfg.inverse_kinematics,
        controller_config);
    auto cfc = std::make_shared<CartesianFollowerController>();
    if (!cfc->init(cfc_cfg))
      throw std::invalid_argument(
          "Failed to init CartesianFollowerController with name: " +
          cfc_cfg.controller_name);
    return cfc;
  }
  throw std::invalid_argument(
      "No valid controller specified with [create_class_name]: " + class_name);
  return nullptr;
}

} // namespace controller
} // namespace medrct

// clang-format off
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::controller::DefaultControllerFactory, DefaultControllerFactory, DCtrlF)
// clang-format on
