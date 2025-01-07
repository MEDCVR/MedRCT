#include <gtest/gtest.h>

#include <medrct_ros/conversions/conversions.hh>
#include <medrct/log.hh>

using namespace medrct_ros;

TEST(ConversionsTest, testJointState)
{
  sensor_msgs::JointState ros_js;
  ros_js.name = {"j1", "j2", "j3"};
  ros_js.position = {0.1, 0.2, 0.3};
  ros_js.velocity = {0.4, 0.5, 0.6};
  ros_js.effort = {0.7, 0.8, 0.9};

  medrct::JointState medrct_js = RosToMedrctJs(ros_js);
  auto ros_js2 = MedrctToRosJs(medrct_js);
  ASSERT_EQ(ros_js.name.size(), ros_js2.name.size());
  ASSERT_EQ(ros_js.position.size(), ros_js2.position.size());
  ASSERT_EQ(ros_js.velocity.size(), ros_js2.velocity.size());
  ASSERT_EQ(ros_js.effort.size(), ros_js2.effort.size());

  for (unsigned int i = 0; i < ros_js.name.size(); ++i)
  {
    EXPECT_EQ(ros_js.name[i], ros_js2.name[i]);
    EXPECT_NEAR(ros_js.position[i], ros_js2.position[i], 10e-6);
    EXPECT_NEAR(ros_js.velocity[i], ros_js2.velocity[i], 10e-6);
    EXPECT_NEAR(ros_js.effort[i], ros_js2.effort[i], 10e-6);
  }
}

TEST(ConversionsTest, testTransform)
{
  geometry_msgs::TransformStamped ros_tf_stamped;
  ros_tf_stamped.transform.translation.x = 0.1;
  ros_tf_stamped.transform.translation.y = 0.2;
  ros_tf_stamped.transform.translation.z = 0.3;
  ros_tf_stamped.transform.rotation.x = 0.7071068;
  ros_tf_stamped.transform.rotation.y = 0.0;
  ros_tf_stamped.transform.rotation.z = 0.0;
  ros_tf_stamped.transform.rotation.w = 0.7071068;

  medrct::Transform medrct_tf = RosToMedrctTf(ros_tf_stamped);
  auto ros_tf_stamped2 = MedrctToRosTf(medrct_tf);

  // Debugging
  //   medrctlog::info("position: \n{}", medrct_tf.translation());
  //   medrctlog::info("rotation: \n{}", medrct_tf.linear());
  //   medrct::Quaternion q(medrct_tf.linear());
  //   medrctlog::info("quaternion: \nw:{} x:{} y:{} z:{}", q.w(), q.x(), q.y(),
  //   q.z()); medrctlog::info("ros pos: \n{}",
  //   ros_tf_stamped2.transform.translation); medrctlog::info("ros quat: \n{}",
  //   ros_tf_stamped2.transform.rotation);

  ASSERT_NEAR(
      ros_tf_stamped.transform.translation.x,
      ros_tf_stamped2.transform.translation.x,
      10e-6);
  ASSERT_NEAR(
      ros_tf_stamped.transform.translation.y,
      ros_tf_stamped2.transform.translation.y,
      10e-6);
  ASSERT_NEAR(
      ros_tf_stamped.transform.translation.z,
      ros_tf_stamped2.transform.translation.z,
      10e-6);
  ASSERT_NEAR(
      ros_tf_stamped.transform.rotation.x,
      ros_tf_stamped2.transform.rotation.x,
      10e-6);
  ASSERT_NEAR(
      ros_tf_stamped.transform.rotation.y,
      ros_tf_stamped2.transform.rotation.y,
      10e-6);
  ASSERT_NEAR(
      ros_tf_stamped.transform.rotation.z,
      ros_tf_stamped2.transform.rotation.z,
      10e-6);
  ASSERT_NEAR(
      ros_tf_stamped.transform.rotation.w,
      ros_tf_stamped2.transform.rotation.w,
      10e-6);
}

TEST(ConversionsTest, testPose)
{
  geometry_msgs::PoseStamped ros_pose_stamped;
  ros_pose_stamped.pose.position.x = 0.1;
  ros_pose_stamped.pose.position.y = 0.2;
  ros_pose_stamped.pose.position.z = 0.3;
  ros_pose_stamped.pose.orientation.x = 0.7071068;
  ros_pose_stamped.pose.orientation.y = 0.0;
  ros_pose_stamped.pose.orientation.z = 0.0;
  ros_pose_stamped.pose.orientation.w = 0.7071068;

  medrct::Transform medrct_tf = RosPoseToMedrctTf(ros_pose_stamped);
  auto ros_pose_stamped2 = MedrctTfToRosPose(medrct_tf);

  // Debugging
  //   medrctlog::info("position: \n{}", medrct_tf.translation());
  //   medrctlog::info("rotation: \n{}", medrct_tf.linear());
  //   medrct::Quaternion q(medrct_tf.linear());
  //   medrctlog::info("quaternion: \nw:{} x:{} y:{} z:{}", q.w(), q.x(), q.y(),
  //   q.z()); medrctlog::info("ros pos: \n{}",
  //   ros_pose_stamped2.transform.translation); medrctlog::info("ros quat: \n{}",
  //   ros_pose_stamped2.transform.rotation);

  ASSERT_NEAR(
      ros_pose_stamped.pose.position.x,
      ros_pose_stamped2.pose.position.x,
      10e-6);
  ASSERT_NEAR(
      ros_pose_stamped.pose.position.y,
      ros_pose_stamped2.pose.position.y,
      10e-6);
  ASSERT_NEAR(
      ros_pose_stamped.pose.position.z,
      ros_pose_stamped2.pose.position.z,
      10e-6);
  ASSERT_NEAR(
      ros_pose_stamped.pose.orientation.x,
      ros_pose_stamped2.pose.orientation.x,
      10e-6);
  ASSERT_NEAR(
      ros_pose_stamped.pose.orientation.y,
      ros_pose_stamped2.pose.orientation.y,
      10e-6);
  ASSERT_NEAR(
      ros_pose_stamped.pose.orientation.z,
      ros_pose_stamped2.pose.orientation.z,
      10e-6);
  ASSERT_NEAR(
      ros_pose_stamped.pose.orientation.w,
      ros_pose_stamped2.pose.orientation.w,
      10e-6);
}

TEST(ConversionsTest, testTwist)
{
  geometry_msgs::TwistStamped ros_twist_stamped;
  ros_twist_stamped.twist.linear.x = 0.1;
  ros_twist_stamped.twist.linear.y = 0.2;
  ros_twist_stamped.twist.linear.z = 0.3;
  ros_twist_stamped.twist.angular.x = 0.4;
  ros_twist_stamped.twist.angular.y = 0.5;
  ros_twist_stamped.twist.angular.z = 0.6;

  medrct::Twist medrct_twist = RosToMedrctTwist(ros_twist_stamped);
  auto ros_twist_stamped2 = MedrctToRosTwist(medrct_twist);

  ASSERT_NEAR(
      ros_twist_stamped.twist.linear.x,
      ros_twist_stamped2.twist.linear.x,
      10e-6);
  ASSERT_NEAR(
      ros_twist_stamped.twist.linear.y,
      ros_twist_stamped2.twist.linear.y,
      10e-6);
  ASSERT_NEAR(
      ros_twist_stamped.twist.linear.z,
      ros_twist_stamped2.twist.linear.z,
      10e-6);
  ASSERT_NEAR(
      ros_twist_stamped.twist.angular.x,
      ros_twist_stamped2.twist.angular.x,
      10e-6);
  ASSERT_NEAR(
      ros_twist_stamped.twist.angular.y,
      ros_twist_stamped2.twist.angular.y,
      10e-6);
  ASSERT_NEAR(
      ros_twist_stamped.twist.angular.z,
      ros_twist_stamped2.twist.angular.z,
      10e-6);
}
