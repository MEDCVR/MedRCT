#include <vector>

#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct/types/twist.hh>

#include <medrct_ros/ros_singleton.hh>
#include <medrct_ros/ros_stream_factory.hh>
#include <medrct_ros/ros_stream.hh>
#include <medrct_ros/conversions/conversions.hh>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

namespace medrct
{
namespace stream
{
Stream::Ptr RosStreamFactory::create(const YAML::Node& config) const
{
  std::string input_or_output_type;
  std::string topic_name;
  std::string name;
  std::string data_type;
  if (YAML::Node n = config["type"])
    input_or_output_type = n.as<std::string>();
  else
    throw std::runtime_error("No [type] key in stream config");
  if (YAML::Node n = config["topic_name"])
    topic_name = n.as<std::string>();
  else
    throw std::runtime_error("No [topic_name] key in stream config");
  if (YAML::Node n = config["name"])
    name = n.as<std::string>();
  else
    throw std::runtime_error("No [name] key in stream config");
  if (YAML::Node n = config["data_type"])
    data_type = n.as<std::string>();
  else
    throw std::runtime_error("No [data_type] key in stream config");

  Stream::Ptr stream;
  std::shared_ptr<ros::NodeHandle> nh =
      RosSingleton::getInstance().getNodeHandle();
  if (!nh)
  {
    throw std::runtime_error(
        "Please first initialize RosSingleton, no node_handle");
  }

  int queue_size = 1000;
  if (YAML::Node n = config["queue_size"])
    queue_size = n.as<int>();
  if (input_or_output_type == "input")
  {
    if (data_type == "JointState")
    {
      stream =
          std::make_shared<RosInputStream<JointState, sensor_msgs::JointState>>(
              name, &medrct_ros::RosToMedrctJs, *nh, topic_name);
    }
    else if (data_type == "Transform")
    {
      stream = std::make_shared<
          RosInputStream<Transform, geometry_msgs::TransformStamped>>(
          name, &medrct_ros::RosToMedrctTf, *nh, topic_name, queue_size);
    }
    else if (data_type == "Twist")
    {
      stream =
          std::make_shared<RosInputStream<Twist, geometry_msgs::TwistStamped>>(
              name, &medrct_ros::RosToMedrctTwist, *nh, topic_name, queue_size);
    }
    else if (data_type == "Joy")
    {
      stream = std::make_shared<RosInputStream<medrct::Joy, sensor_msgs::Joy>>(
          name, &medrct_ros::RosToMedrctJoy, *nh, topic_name, queue_size);
    }
    else
    {
      throw std::runtime_error(
          "No ros input stream of data type [" + data_type + "] supported.");
    }
  }
  else if (input_or_output_type == "output")
  {
    if (data_type == "JointState")
    {
      stream = std::make_shared<
          RosOutputStream<JointState, sensor_msgs::JointState>>(
          name, &medrct_ros::MedrctToRosJs, *nh, topic_name, queue_size);
    }
    else if (data_type == "Transform")
    {
      stream = std::make_shared<
          RosOutputStream<Transform, geometry_msgs::TransformStamped>>(
          name, &medrct_ros::MedrctToRosTf, *nh, topic_name, queue_size);
    }
    else if (data_type == "Twist")
    {
      stream =
          std::make_shared<RosOutputStream<Twist, geometry_msgs::TwistStamped>>(
              name, &medrct_ros::MedrctToRosTwist, *nh, topic_name, queue_size);
    }
    else if (data_type == "Joy")
    {
      stream = std::make_shared<RosOutputStream<medrct::Joy, sensor_msgs::Joy>>(
          name, &medrct_ros::MedrctToRosJoy, *nh, topic_name, queue_size);
    }
    else
    {
      throw std::runtime_error(
          "No ros output stream of data type [" + data_type + "] supported.");
    }
  }
  else
    throw std::runtime_error(
        "Intra stream type can only be 'input', or 'output'");
  return stream;
}
} // namespace stream
} // namespace medrct
