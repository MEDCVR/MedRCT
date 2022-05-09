#pragma once

#include <ros/ros.h>
#include <medrct_common/log.hh>

namespace medrct
{
namespace stream
{
template <typename medrctT, typename rosT>
RosOutputStream<medrctT, rosT>::RosOutputStream(
    const std::string& name,
    const std::function<rosT(const medrctT&)>& medrct_to_ros,
    ros::NodeHandle& n,
    const std::string& topic_name,
    int queue)
    : OutputStream<medrctT>(name), medrct_to_ros(medrct_to_ros)
{
  publisher = n.advertise<rosT>(topic_name, queue);
}
template <typename medrctT, typename rosT>
RosOutputStream<medrctT, rosT>::~RosOutputStream()
{
}

template <typename medrctT, typename rosT>
void RosOutputStream<medrctT, rosT>::publishImpl(const medrctT& data) const
{
  publisher.publish(medrct_to_ros(data));
}

template <typename medrctT, typename rosT>
RosInputStream<medrctT, rosT>::RosInputStream(
    const std::string& name,
    const std::function<medrctT(const rosT&)>& ros_to_medrct,
    ros::NodeHandle& n,
    const std::string& topic_name,
    int queue_size)
    : InputStream<medrctT>(name), ros_to_medrct(ros_to_medrct)
{
  subscriber = n.subscribe(
      topic_name,
      queue_size,
      &RosInputStream<medrctT, rosT>::runCallback,
      this);
}
template <typename medrctT, typename rosT>
RosInputStream<medrctT, rosT>::~RosInputStream()
{
}

template <typename medrctT, typename rosT>
void RosInputStream<medrctT, rosT>::runCallback(const rosT& data)
{
  // I have to use this on a parent member variable which is a template?
  // https://stackoverflow.com/questions/6592512/templates-parent-class-member-variables-not-visible-in-inherited-class
  InputStream<medrctT>::runCallback(ros_to_medrct(data));
}
} // namespace stream
} // namespace medrct
