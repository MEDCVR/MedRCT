#pragma once

#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct/types/twist.hh>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace medrct_ros2
{
inline medrct::JointState RosToMedrctJs(const sensor_msgs::msg::JointState& ros_js)
{
  medrct::JointState medrct_js;
  medrct_js.reserve(ros_js.name.size());
  medrct_js.names = ros_js.name;
  medrct_js.positions = ros_js.position;
  medrct_js.velocities = ros_js.velocity;
  medrct_js.efforts = ros_js.effort;
  return medrct_js;
}

inline sensor_msgs::msg::JointState
MedrctToRosJs(const medrct::JointState& medrct_js)
{
  sensor_msgs::msg::JointState ros_js;
  // ros_js.header TODO
  ros_js.name = medrct_js.names;
  ros_js.position = medrct_js.positions;
  ros_js.velocity = medrct_js.velocities;
  ros_js.effort = medrct_js.efforts;
  return ros_js;
}

inline medrct::Vector3 RosToMedrctVector3(const geometry_msgs::msg::Vector3& ros_vec)
{
  medrct::Vector3 medrct_vec;
  medrct_vec.x() = ros_vec.x;
  medrct_vec.y() = ros_vec.y;
  medrct_vec.z() = ros_vec.z;
  return medrct_vec;
}

inline geometry_msgs::msg::Vector3
MedrctToRosVector3(const medrct::Vector3& medrct_vec)
{
  geometry_msgs::msg::Vector3 ros_vec;
  ros_vec.x = medrct_vec.x();
  ros_vec.y = medrct_vec.y();
  ros_vec.z = medrct_vec.z();
  return ros_vec;
}

inline medrct::Transform
RosToMedrctTf(const geometry_msgs::msg::TransformStamped& ros_tf_stamped)
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

inline geometry_msgs::msg::TransformStamped
MedrctToRosTf(const medrct::Transform& medrct_tf)
{
  geometry_msgs::msg::TransformStamped ros_tf_stamped;
  ros_tf_stamped.transform.translation =
      MedrctToRosVector3(medrct_tf.translation());
  medrct::Quaternion q(medrct_tf.linear());
  ros_tf_stamped.transform.rotation.w = q.w();
  ros_tf_stamped.transform.rotation.x = q.x();
  ros_tf_stamped.transform.rotation.y = q.y();
  ros_tf_stamped.transform.rotation.z = q.z();
  return ros_tf_stamped;
}

inline medrct::Transform
RosPoseToMedrctTf(const geometry_msgs::msg::PoseStamped& ros_pose_stamped)
{
  const auto& ros_pose = ros_pose_stamped.pose;
  medrct::Transform medrct_tf;
  medrct_tf.translation().x() = ros_pose.position.x;
  medrct_tf.translation().y() = ros_pose.position.y;
  medrct_tf.translation().z() = ros_pose.position.z;
  medrct_tf.linear() = medrct::Quaternion(
                           ros_pose.orientation.w,
                           ros_pose.orientation.x,
                           ros_pose.orientation.y,
                           ros_pose.orientation.z)
                           .toRotationMatrix();
  return medrct_tf;
}

inline geometry_msgs::msg::PoseStamped
MedrctTfToRosPose(const medrct::Transform& medrct_tf)
{
  geometry_msgs::msg::PoseStamped ros2_pose_stamped;
  ros2_pose_stamped.pose.position.x = medrct_tf.translation().x();
  ros2_pose_stamped.pose.position.y = medrct_tf.translation().y();
  ros2_pose_stamped.pose.position.z = medrct_tf.translation().z();
  medrct::Quaternion q(medrct_tf.linear());
  ros2_pose_stamped.pose.orientation.w = q.w();
  ros2_pose_stamped.pose.orientation.x = q.x();
  ros2_pose_stamped.pose.orientation.y = q.y();
  ros2_pose_stamped.pose.orientation.z = q.z();
  return ros2_pose_stamped;
}

inline medrct::Twist
RosToMedrctTwist(const geometry_msgs::msg::TwistStamped& ros_twist_stamped)
{
  const auto& ros_twist = ros_twist_stamped.twist;

  medrct::Twist medrct_twist;
  medrct_twist.linear = RosToMedrctVector3(ros_twist.linear);
  medrct_twist.angular = RosToMedrctVector3(ros_twist.angular);
  return medrct_twist;
}

inline geometry_msgs::msg::TwistStamped
MedrctToRosTwist(const medrct::Twist& medrct_twist)
{
  geometry_msgs::msg::TwistStamped ros2_twist_stamped;
  ros2_twist_stamped.twist.linear = MedrctToRosVector3(medrct_twist.linear);
  ros2_twist_stamped.twist.angular = MedrctToRosVector3(medrct_twist.angular);
  return ros2_twist_stamped;
}

inline medrct::Wrench
RosToMedrctWrench(const geometry_msgs::msg::WrenchStamped& ros2_wrench_stamped)
{
  const auto& ros2_wrench = ros2_wrench_stamped.wrench;
  medrct::Wrench medrct_wrench;
  medrct_wrench.force = RosToMedrctVector3(ros2_wrench.force);
  medrct_wrench.torque = RosToMedrctVector3(ros2_wrench.torque);
  return medrct_wrench;
}

inline geometry_msgs::msg::WrenchStamped
MedrctToRosWrench(const medrct::Wrench& medrct_wrench)
{
  geometry_msgs::msg::WrenchStamped ros2_wrench;
  ros2_wrench.wrench.force = MedrctToRosVector3(medrct_wrench.force);
  ros2_wrench.wrench.torque = MedrctToRosVector3(medrct_wrench.torque);
  return ros2_wrench;
}

inline medrct::Joy RosToMedrctJoy(const sensor_msgs::msg::Joy& ros_joy)
{
  medrct::Joy medrct_joy;
  medrct_joy.buttons.reserve(ros_joy.buttons.size());
  for (const auto& button : ros_joy.buttons)
  {
    medrct_joy.buttons.push_back(button);
  }

  return medrct_joy;
}

inline sensor_msgs::msg::Joy MedrctToRosJoy(const medrct::Joy& medrct_joy)
{
  sensor_msgs::msg::Joy ros_joy;
  ros_joy.buttons.reserve(medrct_joy.buttons.size());
  for (const auto& data_bool : medrct_joy.buttons)
  {
    ros_joy.buttons.push_back(data_bool);
  }
  return ros_joy;
}

} // namespace medrct_ros
