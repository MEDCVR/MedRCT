#pragma once

#include <memory>
#include <ros/ros.h>

namespace medrct
{
namespace stream
{
class RosSingleton
{
private:
  RosSingleton();
  ~RosSingleton();
  RosSingleton(const RosSingleton&) = delete;
  RosSingleton& operator=(const RosSingleton&) = delete;

public:
  bool
  init(int argc, char** argv, const std::string& node_name = "medrct_teleop");
  static RosSingleton& getInstance();
  std::shared_ptr<ros::NodeHandle> getNodeHandle();

private:
  std::shared_ptr<ros::NodeHandle> node_handle;
};
} // namespace stream
} // namespace medrct
