#pragma once

#include <medrct_common/joint_state.hh>
#include <medrct_common/types.hh>
#include <medrct_common/twist.hh>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
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

inline medrct::Transform
RosToMedrctTf(const geometry_msgs::TransformStamped& ros_tf_stamped)
{
  const auto& ros_tf = ros_tf_stamped.transform;
  medrct::Transform medrct_tf;
  medrct_tf.translation().x() = ros_tf.translation.x;
  medrct_tf.translation().y() = ros_tf.translation.y;
  medrct_tf.translation().z() = ros_tf.translation.z;

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
  ros_tf_stamped.transform.translation.x = medrct_tf.translation().x();
  ros_tf_stamped.transform.translation.y = medrct_tf.translation().y();
  ros_tf_stamped.transform.translation.z = medrct_tf.translation().z();
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
  const auto& ros_linear = ros_twist.linear;
  const auto& ros_angular = ros_twist.angular;

  medrct::Twist medrct_twist;
  medrct_twist.linear =
      medrct::Vector3(ros_linear.x, ros_linear.y, ros_linear.z);
  medrct_twist.angular =
      medrct::Vector3(ros_angular.x, ros_angular.y, ros_angular.z);
  return medrct_twist;
}

inline geometry_msgs::TwistStamped
MedrctToRosTwist(const medrct::Twist& medrct_twist)
{
  geometry_msgs::TwistStamped ros_twist_stamped;
  ros_twist_stamped.twist.linear.x = medrct_twist.linear.x();
  ros_twist_stamped.twist.linear.y = medrct_twist.linear.y();
  ros_twist_stamped.twist.linear.z = medrct_twist.linear.z();
  ros_twist_stamped.twist.angular.x = medrct_twist.angular.x();
  ros_twist_stamped.twist.angular.y = medrct_twist.angular.y();
  ros_twist_stamped.twist.angular.z = medrct_twist.angular.z();
  return ros_twist_stamped;
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
