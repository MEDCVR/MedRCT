#include <gtest/gtest.h>

#include <medrct_common/twist.hh>
#include <medrct_common/types.hh>

using namespace medrct;

// Declare a test
TEST(TestTwist, constructTwist)
{
  Twist twist;
  twist.linear = Vector3(0, 1, 3);
  twist.angular = Vector3(2, 4, 6);
}
