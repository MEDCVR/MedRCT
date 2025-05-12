#include <vector>

#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct/types/twist.hh>

#include <medrct_ros2/ros2_singleton.hh>
#include <medrct_ros2/ros2_stream_factory.hh>
#include <medrct_ros2/ros2_stream.hh>
#include <medrct_ros2/conversions/conversions.hh>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace medrct
{
namespace stream
{
Stream::Ptr Ros2StreamFactory::create(const YAML::Node& config) const
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
  std::shared_ptr<rclcpp::Node> nh =
      Ros2Singleton::getInstance().getNodeHandle();
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
          std::make_shared<Ros2SubStream<JointState, sensor_msgs::msg::JointState>>(
              name, &medrct_ros2::RosToMedrctJs, *nh, topic_name);
    }
    else if (data_type == "Transform")
    {
      stream = std::make_shared<
          Ros2SubStream<Transform, geometry_msgs::msg::TransformStamped>>(
          name, &medrct_ros2::RosToMedrctTf, *nh, topic_name);
    }
    else if (data_type == "Pose")
    {
      stream =
          std::make_shared<Ros2SubStream<Transform, geometry_msgs::msg::PoseStamped>>(
              name,
              &medrct_ros2::RosPoseToMedrctTf,
              *nh,
              topic_name);
    }
    else if (data_type == "Twist")
    {
      stream =
          std::make_shared<Ros2SubStream<Twist, geometry_msgs::msg::TwistStamped>>(
              name, &medrct_ros2::RosToMedrctTwist, *nh, topic_name);
    }
    else if (data_type == "Joy")
    {
      stream = std::make_shared<Ros2SubStream<medrct::Joy, sensor_msgs::msg::Joy>>(
          name, &medrct_ros2::RosToMedrctJoy, *nh, topic_name);
    }
    else if (data_type == "Wrench")
    {
      stream = std::make_shared<
          Ros2SubStream<medrct::Wrench, geometry_msgs::msg::WrenchStamped>>(
          name, &medrct_ros2::RosToMedrctWrench, *nh, topic_name);
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
          Ros2PubStream<medrct::JointState, sensor_msgs::msg::JointState>>(
          name, &medrct_ros2::MedrctToRosJs, *nh, topic_name, queue_size);
    }
    else if (data_type == "Transform")
    {
      stream = std::make_shared<
          Ros2PubStream<medrct::Transform, geometry_msgs::msg::TransformStamped>>(
          name, &medrct_ros2::MedrctToRosTf, *nh, topic_name, queue_size);
    }
    else if (data_type == "Pose")
    {
      stream = std::make_shared<
          Ros2PubStream<medrct::Transform, geometry_msgs::msg::PoseStamped>>(
          name, &medrct_ros2::MedrctTfToRosPose, *nh, topic_name, queue_size);
    }
    else if (data_type == "Twist")
    {
      stream = std::make_shared<
          Ros2PubStream<medrct::Twist, geometry_msgs::msg::TwistStamped>>(
          name, &medrct_ros2::MedrctToRosTwist, *nh, topic_name, queue_size);
    }
    else if (data_type == "Joy")
    {
      stream = std::make_shared<Ros2PubStream<medrct::Joy, sensor_msgs::msg::Joy>>(
          name, &medrct_ros2::MedrctToRosJoy, *nh, topic_name, queue_size);
    }
    else if (data_type == "Wrench")
    {
      stream = std::make_shared<
          Ros2PubStream<medrct::Wrench, geometry_msgs::msg::WrenchStamped>>(
          name, &medrct_ros2::MedrctToRosWrench, *nh, topic_name, queue_size);
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
