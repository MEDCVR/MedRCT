#pragma once

#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct_common/interface/stream_factory.hh>
#include <medrct_common/joint_state.hh>
#include <medrct_controller/controller.hh>
namespace medrct
{
namespace env
{
class ForwardKinematics;
class InverseKinematics;
} // namespace env
namespace controller
{

class ControllerFactory
{
public:
  ControllerFactory(stream::StreamFactory::ConstPtr stream_factory);
  virtual ~ControllerFactory() = default;
  virtual ControllerInterface::Ptr create(YAML::Node config) const;

protected:
  void getControllerName(
      std::string& name, const YAML::Node& controller_config) const;
  void createOutputAndMeasuredStreams(
      stream::OutputStream<JointState>::Ptr& output_js_stream,
      stream::InputStream<JointState>::Ptr& measured_js_stream,
      const YAML::Node& controller_config) const;
  void createKinematicsSolvers(
      std::shared_ptr<env::ForwardKinematics>& forward_kinematics,
      std::shared_ptr<env::InverseKinematics>& inverse_kinematics,
      const YAML::Node& controller_config) const;
  const stream::StreamFactory::ConstPtr stream_factory;
};

} // namespace controller
} // namespace medrct
