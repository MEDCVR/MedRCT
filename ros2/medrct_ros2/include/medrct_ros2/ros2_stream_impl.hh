#pragma once

#include <rclcpp/rclcpp.hpp>
#include <medrct/log.hh>

namespace medrct
{
namespace stream
{
template <typename medrctT, typename ros2T>
Ros2PubStream<medrctT, ros2T>::Ros2PubStream(
    const std::string& name,
    const std::function<ros2T(const medrctT&)>& medrct_to_ros2,
    rclcpp::Node& n,
    const std::string& topic_name,
    int queue)
    : PubStream<medrctT>(name), medrct_to_ros2(medrct_to_ros2)
{
  publisher = n.create_publisher<ros2T>(topic_name, queue);
}
template <typename medrctT, typename ros2T>
Ros2PubStream<medrctT, ros2T>::~Ros2PubStream()
{
}

template <typename medrctT, typename ros2T>
void Ros2PubStream<medrctT, ros2T>::publishImpl(const medrctT& data) const
{
  publisher->publish(medrct_to_ros2(data));
}

template <typename medrctT, typename ros2T>
Ros2SubStream<medrctT, ros2T>::Ros2SubStream(
    const std::string& name,
    const std::function<medrctT(const ros2T&)>& ros2_to_medrct,
    rclcpp::Node& n,
    const std::string& topic_name)
    : SubStream<medrctT>(name), ros2_to_medrct(ros2_to_medrct)
{

  auto my_callback_group = n.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = my_callback_group;
  subscriber = n.create_subscription<ros2T>(
      topic_name, rclcpp::SensorDataQoS(),
      std::bind(&Ros2SubStream<medrctT, ros2T>::runCallback, this, std::placeholders::_1),
      options);
}
template <typename medrctT, typename ros2T>
Ros2SubStream<medrctT, ros2T>::~Ros2SubStream()
{
}

template <typename medrctT, typename ros2T>
void Ros2SubStream<medrctT, ros2T>::runCallback(const std::shared_ptr<ros2T> data)
{
  // I have to use this on a parent member variable which is a template?
  // https://stackoverflow.com/questions/6592512/templates-parent-class-member-variables-not-visible-in-inherited-class
  SubStream<medrctT>::runCallback(ros2_to_medrct(*data));
}
} // namespace stream
} // namespace medrct
