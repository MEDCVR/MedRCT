#include <gtest/gtest.h>

#include <medrct_common/joint_state.hh>

using namespace medrct;

// Declare a test
TEST(TestJointState, constructJointState)
{
  JointState joint_state;
  joint_state.reserve(1);
  joint_state.push_back("radian", 0.1);

  EXPECT_EQ(joint_state.names[0], "radian");
  EXPECT_EQ(joint_state.positions[0], 0.1);
}
