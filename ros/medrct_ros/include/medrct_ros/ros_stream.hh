#pragma once

#include <functional>

#include <ros/ros.h>
#include <medrct/stream/stream.hh>

namespace medrct
{
namespace stream
{
template <typename medrctT, typename rosT>
class RosOutputStream : public OutputStream<medrctT>
{
public:
  RosOutputStream(
      const std::string& name,
      const std::function<rosT(const medrctT&)>& medrct_to_ros,
      ros::NodeHandle& n,
      const std::string& topic_name,
      int queue = 10);
  virtual ~RosOutputStream();

private:
  virtual void publishImpl(const medrctT& data) const override;
  std::function<rosT(const medrctT)> medrct_to_ros;
  ros::Publisher publisher;
};

template <typename medrctT, typename rosT>
class RosInputStream : public InputStream<medrctT>
{
public:
  RosInputStream(
      const std::string& name,
      const std::function<medrctT(const rosT&)>& ros_to_medrct,
      ros::NodeHandle& n,
      const std::string& topic_name,
      int queue_size = 1000);
  virtual ~RosInputStream();

private:
  void runCallback(const rosT& data);
  std::function<medrctT(const rosT)> ros_to_medrct;
  ros::Subscriber subscriber;
};
} // namespace stream
} // namespace medrct

#include "ros_stream_impl.hh"
