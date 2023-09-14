#include <gtest/gtest.h>

#include <medrct/types/types.hh>
#include <medrct_env/description/joint.hh>
#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/description/link.hh>
#include <medrct_env/kinematics/simple_forward_kinematics.hh>

TEST(TestSimpleFK, testOutput)
{
  using namespace medrct;
  using namespace medrct::env;

  real_t tol = 10e-5;

  // Make a 2 DOF robot
  KinematicsTree kin_tree;
  kin_tree.addLink(Link("base_link"));
  {
    kin_tree.addLink(Link("first_link"));
    Joint joint("first_joint");
    joint.parent_to_joint_origin_transform.translation() = Vector3(0, 0, 0);
    joint.axis = Vector3::UnitX();
    joint.type = JointType::REVOLUTE;
    joint.parent_link_name = "base_link";
    joint.child_link_name = "first_link";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("second_link"));
    Joint joint("second_joint");
    joint.parent_to_joint_origin_transform.translation() = Vector3(0, 0, 1);
    joint.axis = Vector3::UnitX();
    joint.type = JointType::REVOLUTE;
    joint.parent_link_name = "first_link";
    joint.child_link_name = "second_link";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("tip_link"));
    Joint joint("tip_joint");
    joint.parent_to_joint_origin_transform.translation() = Vector3(0, 0, 1);
    joint.type = JointType::FIXED;
    joint.parent_link_name = "second_link";
    joint.child_link_name = "tip_link";
    kin_tree.addJoint(joint);
  }

  // Instantiate simple fk
  Chain chain;
  bool chain_init_ret = chain.init(kin_tree, "base_link", "tip_link");
  ASSERT_EQ(chain_init_ret, true);
  SimpleForwardKinematics simple_fk(chain);

  // Calculate FK
  Transform transform_out;
  int compute_fk_ret = simple_fk.computeFK(transform_out, {0, 0});
  ASSERT_EQ(compute_fk_ret, 0);
  ASSERT_NEAR(transform_out.translation().z(), 2.0, tol);

  compute_fk_ret = simple_fk.computeFK(transform_out, {0, PI_2});
  ASSERT_EQ(compute_fk_ret, 0);
  ASSERT_NEAR(transform_out.translation().z(), 1.0, tol);
  ASSERT_NEAR(transform_out.translation().y(), -1.0, tol);

  // Calculate FK of link
  compute_fk_ret = simple_fk.computeFK(transform_out, {0, 0}, 2);
  ASSERT_NEAR(transform_out.translation().z(), 1.0, tol);
  ASSERT_NEAR(transform_out.translation().y(), 0.0, tol);

  // Calculate FK of first link
  compute_fk_ret = simple_fk.computeFK(transform_out, {0}, 1);
  ASSERT_NEAR(transform_out.translation().z(), 0.0, tol);
  ASSERT_NEAR(transform_out.translation().y(), 0.0, tol);
}
