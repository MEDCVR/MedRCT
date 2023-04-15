#include <string>

#include <medrct/loader/class_loader.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/log.hh>
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
  if (YAML::Node n = controller_config["output_stream"])
  {
    n["name"] =
        controller_config["name"].as<std::string>() + "_output_js_stream";
    n["type"] = "output";
    n["data_type"] = "JointState";
    output_js_stream = stream_factory.create<stream::PubStream<JointState>>(n);
  }
  else
    throw std::runtime_error("No [output_stream] in controller config");
  if (YAML::Node n = controller_config["feedback_stream"])
  {
    n["name"] =
        controller_config["name"].as<std::string>() + "_feedback_js_stream";
    n["type"] = "input";
    n["data_type"] = "JointState";
    measured_js_stream =
        stream_factory.create<stream::SubStream<JointState>>(n);
  }
  else
    throw std::runtime_error("No [feedback_stream] in controller config.");
  return;
}

void CreateKinematicsSolvers(
    std::shared_ptr<env::ForwardKinematics>& forward_kinematics,
    std::shared_ptr<env::InverseKinematics>& inverse_kinematics,
    const YAML::Node& controller_config)
{
  if (!controller_config["kinematics_tree"])
  {
    throw std::runtime_error("No [kinematics_tree] key in controller config.");
  }

  auto kin_tree = env::World::getInstance().getKinematicsTree(
      controller_config["kinematics_tree"].as<std::string>());
  if (!kin_tree)
  {
    medrctlog::warn(
        "Kinematics tree not found, will try creating kinematics however.");
  }

  if (!controller_config["forward_kinematics"])
  {
    throw std::runtime_error(
        "No [forward_kinematics] key in controller config.");
  }
  env::ForwardKinematicsFactory::Ptr forward_kinematics_factory;
  try
  {
    forward_kinematics_factory =
        medrct_loader::createSharedInstance<env::ForwardKinematicsFactory>(
            controller_config["forward_kinematics"]);
    // This function throws if not created
    forward_kinematics = forward_kinematics_factory->create(
        *kin_tree, controller_config["forward_kinematics"]);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "Error in instantiating forward kinematics: " + std::string(e.what()));
  }

  if (!controller_config["inverse_kinematics"])
  {
    throw std::runtime_error(
        "No [inverse_kinematics] key in controller config.");
  }
  env::InverseKinematicsFactory::Ptr inverse_kinematics_factory;
  try
  {
    inverse_kinematics_factory =
        medrct_loader::createSharedInstance<env::InverseKinematicsFactory>(
            controller_config["inverse_kinematics"]);
    // This function throws if not created
    inverse_kinematics = inverse_kinematics_factory->create(
        *kin_tree, controller_config["inverse_kinematics"]);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "Error in inverse_kinematics config: " + *e.what());
  }
}

Controller::Ptr DefaultControllerFactory::create(
    const medrct::stream::StreamFactory& stream_factory,
    YAML::Node controller_config) const
{
  std::string class_name;
  if (YAML::Node n = controller_config["class_name"])
    class_name = n.as<std::string>();
  else
    throw std::runtime_error("No [class_name] key in controller config.");

  if (class_name == "JointMimicController" ||
      class_name == "JointIncrementController")
  {
    JointTeleopControllerConfig jcc;
    getControllerName(jcc.controller_name, controller_config);
    if (YAML::Node n = controller_config["input_stream"])
    {
      n["name"] = jcc.controller_name + "_input_stream";
      n["type"] = "input";
      n["data_type"] = "JointState";
      jcc.input_js_stream =
          stream_factory.create<stream::SubStream<JointState>>(n);
    }
    else
      throw std::runtime_error("No [input_stream] in controller config");
    CreateOutputAndMeasuredStreams(
        jcc.output_js_stream,
        jcc.measured_js_stream,
        controller_config,
        stream_factory);

    if (class_name == "JointMimicController")
    {
      auto jmc = std::make_shared<JointMimicController>();
      if (!jmc->init(jcc))
      {
        throw std::runtime_error(
            "Failed to init JointMimicController with name: ");
      }
      return jmc;
    }
    else // Must be JointIncrementController
    {
      auto jic = std::make_shared<JointIncrementController>();
      if (!jic->init(jcc))
      {
        throw std::runtime_error(
            "Failed to init JointIncrementController with name: ");
      }
      return jic;
    }
  }
  else if (class_name == "CartesianIncrementController")
  {
    CartesianIncrementControllerConfig cic_cfg;
    getControllerName(cic_cfg.controller_name, controller_config);
    if (YAML::Node n = controller_config["input_stream"])
    {
      n["name"] = cic_cfg.controller_name + "_input_stream";
      n["type"] = "input";
      n["data_type"] = "Twist";
      cic_cfg.input_callback_stream =
          stream_factory.create<stream::SubStream<Twist>>(n);
    }
    else
      throw std::runtime_error("No [input_stream] in controller config");
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
    {
      throw std::runtime_error(
          "Failed to init CartesianIncrementController with name: " +
          cic_cfg.controller_name);
    }
    return cic;
  }
  else if (class_name == "CartesianFollowerController")
  {
    CartesianFollowerControllerConfig cfc_cfg;
    getControllerName(cfc_cfg.controller_name, controller_config);
    if (YAML::Node n = controller_config["input_stream"])
    {
      n["name"] = cfc_cfg.controller_name + "_input_stream";
      n["type"] = "input";
      n["data_type"] = "Transform";
      cfc_cfg.input_callback_stream =
          stream_factory.create<stream::SubStream<Transform>>(n);
    }
    else
      throw std::runtime_error("No [input_stream] in controller config");
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
    {
      throw std::runtime_error(
          "Failed to init CartesianFollowerController with name: ");
    }
    return cfc;
  }
  throw std::runtime_error(
      "No valid controller specified with class_name: " + class_name);
  return nullptr;
}

} // namespace controller
} // namespace medrct

// clang-format off
MEDRCT_ADD_PLUGIN_SECTIONED(medrct::controller::DefaultControllerFactory, DefaultControllerFactory, DCtrlF)
// clang-format on
