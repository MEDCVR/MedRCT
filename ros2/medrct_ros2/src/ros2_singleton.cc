#include <medrct/stream/condition_wait.hh>
#include <medrct/log.hh>

#include <medrct_ros2/ros2_singleton.hh>

namespace medrct
{
namespace stream
{

bool Ros2Singleton::init(int argc, char** argv, const std::string& node_name)
{
  if (!node_handle)
  {
    rclcpp::init(argc, argv);
    node_handle = std::make_shared<rclcpp::Node>(node_name);
    if (!rclcpp::ok()) // If Failed to contact ros master, and user ctrl+c
    {
      return false;
    }
    ConditionWait::getInstance().setIsRunningFunction(
        []() { return rclcpp::ok(); });
  }
  return true;
}

Ros2Singleton& Ros2Singleton::getInstance()
{
  static Ros2Singleton r;
  return r;
}

std::shared_ptr<rclcpp::Node> Ros2Singleton::getNodeHandle()
{
  if (!node_handle)
  {
    medrctlog::error("Please first initialize Ros2Singleton, no node_handle");
    return nullptr;
  }
  return node_handle;
}
} // namespace stream

} // namespace medrct
