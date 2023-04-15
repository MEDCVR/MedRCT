#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <medrct/types/joint_state.hh>
#include <medrct/stream/stream.hh>
#include <medrct_ros/conversions/conversions.hh>
#include <medrct_ros/ros_stream.hh>

/*
rostopic echo /joint_state
*/
using namespace medrct;
using namespace medrct::stream;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle node_handle;

  // Put as the abstract base class pointer PubStream
  std::shared_ptr<PubStream<JointState>> output_stream =
      std::make_shared<RosPubStream<JointState, sensor_msgs::JointState>>(
          "output_stream",
          &medrct_ros::MedrctToRosJs,
          node_handle,
          "/joint_state");

  ros::Rate loop_rate(10);
  double count = 0.0;
  while (ros::ok())
  {
    // Publish ros messages using the medrct_common data types
    JointState joint_state;
    joint_state.push_back("joint_1", count);
    output_stream->publish(joint_state);
    count = count + 1.0;
    loop_rate.sleep();
  }

  return 0;
}
