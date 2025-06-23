#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace medrct
{
namespace stream
{
class Ros2Singleton
{
private:
  Ros2Singleton() {}
  ~Ros2Singleton() {}
  Ros2Singleton(const Ros2Singleton&) = delete;
  Ros2Singleton& operator=(const Ros2Singleton&) = delete;

public:
  bool
  init(int argc, char** argv, const std::string& node_name = "medrct_teleop");
  static Ros2Singleton& getInstance();
  std::shared_ptr<rclcpp::Node> getNodeHandle();

private:
  std::shared_ptr<rclcpp::Node> node_handle;
};
} // namespace stream
} // namespace medrct
