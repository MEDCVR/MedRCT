#pragma once

#include <medrct/config.hh>

#include <medrct/stream/stream.hh>
#include <medrct/stream/stream_factory.hh>
#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct/loader/class_loader.hh>

#include <medrct_env/description/world.hh>
#include <medrct_env/kinematics/kinematics_factory.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>

namespace medrct
{
namespace controller
{
  inline Quaternion GetYamlQuaternionDefault(YAML::Node config, const std::string& key, const Quaternion& default_quat)
  {
    if(!config[key]) return default_quat;
    if(!config[key]["quaternion"]) return default_quat;
    YAML::Node quaternion_yaml = config[key]["quaternion"];
    Quaternion quat;
    quat.w() = GetValue<real_t>(quaternion_yaml, "w");
    quat.x() = GetValue<real_t>(quaternion_yaml, "x");
    quat.y() = GetValue<real_t>(quaternion_yaml, "y");
    quat.z() = GetValue<real_t>(quaternion_yaml, "x");
    return quat;
  }

  inline void CreateOutputAndMeasuredStreams(
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

  inline void CreateKinematicsSolvers(
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
}

}
