#include <iostream>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <medrct_common/joint_state.hh>
#include <medrct_common/log.hh>

#include <medrct_ros/conversions/conversions.hh>
#include <medrct_ros/ros_stream.hh>

/*
rostopic pub /joint_state sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['first_joint']
position: [0]
velocity: [0]
effort: [0]"
*/

using namespace medrct;
using namespace medrct::stream;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle node_handle;

  // Store as a pointer to the base class to keep abstract interfaces
  std::shared_ptr<InputStream<JointState>> input_buffer_stream =
      std::make_shared<RosInputStream<JointState, sensor_msgs::JointState>>(
          "input_buffer_stream",
          &medrct_ros::RosToMedrctJs,
          node_handle,
          "/joint_state");
  input_buffer_stream->addBuffer();
  const auto& buffer = input_buffer_stream->getBuffer();

  std::thread spin_thread([]() { ros::spin(); });

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    if (!buffer.isEmpty())
    {
      medrctlog::info("joint_name[0]: {}", buffer.getLatest().names[0]);
    }
    else
    {
      medrctlog::info("joint_state is still empty, wait for message");
    }
    loop_rate.sleep();
  }
  spin_thread.join();
  return 0;
}
