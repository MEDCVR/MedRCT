#pragma once

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <medrct/stream/stream.hh>

namespace medrct
{
namespace stream
{
template <typename medrctT, typename ros2T>
class Ros2PubStream : public PubStream<medrctT>
{
public:
  Ros2PubStream(
      const std::string& name,
      const std::function<ros2T(const medrctT&)>& medrct_to_ros2,
      rclcpp::Node& n,
      const std::string& topic_name,
      int queue = 10);
  virtual ~Ros2PubStream();

private:
  virtual void publishImpl(const medrctT& data) const override;
  std::function<ros2T(const medrctT&)> medrct_to_ros2;
  std::shared_ptr<rclcpp::Publisher<ros2T>> publisher;
};

template <typename medrctT, typename ros2T>
class Ros2SubStream : public SubStream<medrctT>
{
public:
  Ros2SubStream(
      const std::string& name,
      const std::function<medrctT(const ros2T&)>& ros2_to_medrct,
      rclcpp::Node& n,
      const std::string& topic_name,
      int queue_size = 1000);
  virtual ~Ros2SubStream();

private:
  void runCallback(const std::shared_ptr<ros2T> data);
  std::function<medrctT(const ros2T&)> ros2_to_medrct;
  std::shared_ptr<rclcpp::Subscription<ros2T>> subscriber;
};
} // namespace stream
} // namespace medrct

#include "ros2_stream_impl.hh"
