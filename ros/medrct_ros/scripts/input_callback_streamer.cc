#include <memory>

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

// Create your subscriber callback logic without specific ROS datatypes (using
// medrct common datatypes)
void SubscriberCallback(const JointState& joint_state)
{
  medrctlog::info("joint_name [0]: {}", joint_state.names[0]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle node_handle;

  // Store as a pointer to the base class to keep abstract interfaces
  std::shared_ptr<InputStream<JointState>> input_callback_stream =
      std::make_shared<RosInputStream<JointState, sensor_msgs::JointState>>(
          "input_callback_stream",
          &medrct_ros::RosToMedrctJs,
          node_handle,
          "/joint_state");

  // Thread safe attach later
  input_callback_stream->addCallback(
      "joint_state_callback", &SubscriberCallback);

  ros::spin();
  return 0;
}
