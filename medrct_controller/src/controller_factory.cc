#include <string>

#include <medrct_common/loader/class_loader.hh>
#include <medrct_common/log.hh>
#include <medrct_controller/controller_factory.hh>
#include <medrct_controller/controllers/cartesian_teleop_controller.hh>
#include <medrct_controller/controllers/joint_teleop_controller.hh>
#include <medrct_environment/kinematics/kinematics_factory.hh>
#include <medrct_environment/kinematics/forward_kinematics.hh>
#include <medrct_environment/kinematics/inverse_kinematics.hh>
#include <medrct_environment/description/world.hh>

namespace medrct
{
namespace controller
{

ControllerFactory::ControllerFactory(
    stream::StreamFactory::ConstPtr stream_factory)
    : stream_factory(stream_factory)
{
  if (!stream_factory)
    throw std::runtime_error("Stream factory is a nullptr.");
}

void ControllerFactory::getControllerName(
    std::string& name, const YAML::Node& controller_config) const
{
  if (YAML::Node n = controller_config["name"])
    name = n.as<std::string>();
  else
    throw std::runtime_error("No [name] in controller config.");
  return;
}

void ControllerFactory::createOutputAndMeasuredStreams(
    stream::OutputStream<JointState>::Ptr& output_js_stream,
    stream::InputStream<JointState>::Ptr& measured_js_stream,
    const YAML::Node& controller_config) const
{
  if (YAML::Node n = controller_config["output_stream"])
  {
    n["name"] =
        controller_config["name"].as<std::string>() + "_output_js_stream";
    n["type"] = "output";
    n["data_type"] = "JointState";
    output_js_stream =
        stream_factory->create<stream::OutputStream<JointState>>(n);
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
        stream_factory->create<stream::InputStream<JointState>>(n);
  }
  else
    throw std::runtime_error("No [feedback_stream] in controller config.");
  return;
}

void ControllerFactory::createKinematicsSolvers(
    env::ForwardKinematics::Ptr& forward_kinematics,
    env::InverseKinematics::Ptr& inverse_kinematics,
    const YAML::Node& controller_config) const
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

ControllerInterface::Ptr
ControllerFactory::create(YAML::Node controller_config) const
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
          stream_factory->create<stream::InputStream<JointState>>(n);
    }
    else
      throw std::runtime_error("No [input_stream] in controller config");
    createOutputAndMeasuredStreams(
        jcc.output_js_stream, jcc.measured_js_stream, controller_config);

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
          stream_factory->create<stream::InputStream<Twist>>(n);
    }
    else
      throw std::runtime_error("No [input_stream] in controller config");
    createOutputAndMeasuredStreams(
        cic_cfg.output_js_stream,
        cic_cfg.measured_js_stream,
        controller_config);
    createKinematicsSolvers(
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
          stream_factory->create<stream::InputStream<Transform>>(n);
    }
    else
      throw std::runtime_error("No [input_stream] in controller config");
    createOutputAndMeasuredStreams(
        cfc_cfg.output_js_stream,
        cfc_cfg.measured_js_stream,
        controller_config);
    createKinematicsSolvers(
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
