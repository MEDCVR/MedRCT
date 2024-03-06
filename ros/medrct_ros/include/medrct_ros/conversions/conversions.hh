#pragma once

#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct/types/twist.hh>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

namespace medrct_ros
{
inline medrct::JointState RosToMedrctJs(const sensor_msgs::JointState& ros_js)
{
  medrct::JointState medrct_js;
  medrct_js.reserve(ros_js.name.size());
  medrct_js.names = ros_js.name;
  medrct_js.positions = ros_js.position;
  medrct_js.velocities = ros_js.velocity;
  medrct_js.efforts = ros_js.effort;
  return medrct_js;
}

inline sensor_msgs::JointState
MedrctToRosJs(const medrct::JointState& medrct_js)
{
  sensor_msgs::JointState ros_js;
  // ros_js.header TODO
  ros_js.name = medrct_js.names;
  ros_js.position = medrct_js.positions;
  ros_js.velocity = medrct_js.velocities;
  ros_js.effort = medrct_js.efforts;
  return ros_js;
}

inline medrct::Vector3 RosToMedrctVector3(const geometry_msgs::Vector3& ros_vec)
{
  medrct::Vector3 medrct_vec;
  medrct_vec.x() = ros_vec.x;
  medrct_vec.y() = ros_vec.y;
  medrct_vec.z() = ros_vec.z;
  return medrct_vec;
}

inline geometry_msgs::Vector3
MedrctToRosVector3(const medrct::Vector3& medrct_vec)
{
  geometry_msgs::Vector3 ros_vec;
  ros_vec.x = medrct_vec.x();
  ros_vec.y = medrct_vec.y();
  ros_vec.z = medrct_vec.z();
  return ros_vec;
}

inline medrct::Transform
RosToMedrctTf(const geometry_msgs::TransformStamped& ros_tf_stamped)
{
  const auto& ros_tf = ros_tf_stamped.transform;
  medrct::Transform medrct_tf;
  medrct_tf.translation() = RosToMedrctVector3(ros_tf.translation);

  medrct_tf.linear() = medrct::Quaternion(
                           ros_tf.rotation.w,
                           ros_tf.rotation.x,
                           ros_tf.rotation.y,
                           ros_tf.rotation.z)
                           .toRotationMatrix();
  return medrct_tf;
}

inline geometry_msgs::TransformStamped
MedrctToRosTf(const medrct::Transform& medrct_tf)
{
  geometry_msgs::TransformStamped ros_tf_stamped;
  ros_tf_stamped.transform.translation =
      MedrctToRosVector3(medrct_tf.translation());
  medrct::Quaternion q(medrct_tf.linear());
  ros_tf_stamped.transform.rotation.w = q.w();
  ros_tf_stamped.transform.rotation.x = q.x();
  ros_tf_stamped.transform.rotation.y = q.y();
  ros_tf_stamped.transform.rotation.z = q.z();
  return ros_tf_stamped;
}

inline medrct::Twist
RosToMedrctTwist(const geometry_msgs::TwistStamped& ros_twist_stamped)
{
  const auto& ros_twist = ros_twist_stamped.twist;

  medrct::Twist medrct_twist;
  medrct_twist.linear = RosToMedrctVector3(ros_twist.linear);
  medrct_twist.angular = RosToMedrctVector3(ros_twist.angular);
  return medrct_twist;
}

inline geometry_msgs::TwistStamped
MedrctToRosTwist(const medrct::Twist& medrct_twist)
{
  geometry_msgs::TwistStamped ros_twist_stamped;
  ros_twist_stamped.twist.linear = MedrctToRosVector3(medrct_twist.linear);
  ros_twist_stamped.twist.angular = MedrctToRosVector3(medrct_twist.angular);
  return ros_twist_stamped;
}

inline medrct::Wrench
RosToMedrctWrench(const geometry_msgs::WrenchStamped& ros_wrench_stamped)
{
  const auto& ros_wrench = ros_wrench_stamped.wrench;
  medrct::Wrench medrct_wrench;
  medrct_wrench.force = RosToMedrctVector3(ros_wrench.force);
  medrct_wrench.torque = RosToMedrctVector3(ros_wrench.torque);
  return medrct_wrench;
}

inline geometry_msgs::WrenchStamped
MedrctToRosWrench(const medrct::Wrench& medrct_wrench)
{
  geometry_msgs::WrenchStamped ros_wrench;
  ros_wrench.wrench.force = MedrctToRosVector3(medrct_wrench.force);
  ros_wrench.wrench.torque = MedrctToRosVector3(medrct_wrench.torque);
  return ros_wrench;
}

inline medrct::Joy RosToMedrctJoy(const sensor_msgs::Joy& ros_joy)
{
  medrct::Joy medrct_joy;
  medrct_joy.buttons.reserve(ros_joy.buttons.size());
  for (const auto& button : ros_joy.buttons)
  {
    medrct_joy.buttons.push_back(button);
  }

  return medrct_joy;
}

inline sensor_msgs::Joy MedrctToRosJoy(const medrct::Joy& medrct_joy)
{
  sensor_msgs::Joy ros_joy;
  ros_joy.buttons.reserve(medrct_joy.buttons.size());
  for (const auto& data_bool : medrct_joy.buttons)
  {
    ros_joy.buttons.push_back(data_bool);
  }
  return ros_joy;
}

} // namespace medrct_ros
