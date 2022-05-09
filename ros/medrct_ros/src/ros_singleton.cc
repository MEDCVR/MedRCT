#include <medrct_common/interface/condition_wait.hh>
#include <medrct_common/log.hh>

#include <medrct_ros/ros_singleton.hh>

namespace medrct
{
namespace stream
{
RosSingleton::RosSingleton()
{
}
RosSingleton::~RosSingleton()
{
}

bool RosSingleton::init(int argc, char** argv, const std::string& node_name)
{
  if (!node_handle)
  {
    ros::init(argc, argv, node_name);
    node_handle = std::make_shared<ros::NodeHandle>();
    if (!ros::ok()) // If Failed to contact ros master, and user ctrl+c
    {
      return false;
    }
    ConditionWait::getInstance().setIsRunningFunction(
        []() { return ros::ok(); });
  }
  return true;
}

RosSingleton& RosSingleton::getInstance()
{
  static RosSingleton r;
  return r;
}

std::shared_ptr<ros::NodeHandle> RosSingleton::getNodeHandle()
{
  if (!node_handle)
  {
    medrctlog::error("Please first initialize RosSingleton, no node_handle");
    return nullptr;
  }
  return node_handle;
}
} // namespace stream

} // namespace medrct
