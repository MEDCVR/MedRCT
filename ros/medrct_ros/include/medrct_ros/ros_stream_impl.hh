#pragma once

#include <ros/ros.h>
#include <medrct/log.hh>

namespace medrct
{
namespace stream
{
template <typename medrctT, typename rosT>
RosPubStream<medrctT, rosT>::RosPubStream(
    const std::string& name,
    const std::function<rosT(const medrctT&)>& medrct_to_ros,
    ros::NodeHandle& n,
    const std::string& topic_name,
    int queue)
    : PubStream<medrctT>(name), medrct_to_ros(medrct_to_ros)
{
  publisher = n.advertise<rosT>(topic_name, queue);
}
template <typename medrctT, typename rosT>
RosPubStream<medrctT, rosT>::~RosPubStream()
{
}

template <typename medrctT, typename rosT>
void RosPubStream<medrctT, rosT>::publishImpl(const medrctT& data) const
{
  publisher.publish(medrct_to_ros(data));
}

template <typename medrctT, typename rosT>
RosSubStream<medrctT, rosT>::RosSubStream(
    const std::string& name,
    const std::function<medrctT(const rosT&)>& ros_to_medrct,
    ros::NodeHandle& n,
    const std::string& topic_name,
    int queue_size)
    : SubStream<medrctT>(name), ros_to_medrct(ros_to_medrct)
{
  subscriber = n.subscribe(
      topic_name, queue_size, &RosSubStream<medrctT, rosT>::runCallback, this);
}
template <typename medrctT, typename rosT>
RosSubStream<medrctT, rosT>::~RosSubStream()
{
}

template <typename medrctT, typename rosT>
void RosSubStream<medrctT, rosT>::runCallback(const rosT& data)
{
  // I have to use this on a parent member variable which is a template?
  // https://stackoverflow.com/questions/6592512/templates-parent-class-member-variables-not-visible-in-inherited-class
  SubStream<medrctT>::runCallback(ros_to_medrct(data));
}
} // namespace stream
} // namespace medrct
