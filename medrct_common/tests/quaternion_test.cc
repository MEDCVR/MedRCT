#include <gtest/gtest.h>

#include <medrct_common/types.hh>
#include <medrct_common/log.hh>

using namespace medrct;

// Declare a test
TEST(TestQuaternion, constructor)
{
  Quaternion quat;
  quat.w() = 1.0;
  quat.x() = 2.0;
  quat.y() = 3.0;
  quat.z() = 4.0;
  EXPECT_EQ(quat.w(), 1.0);
  EXPECT_EQ(quat.x(), 2.0);
  EXPECT_EQ(quat.y(), 3.0);
  EXPECT_EQ(quat.z(), 4.0);
}

TEST(TestQuaternion, multiplication)
{
  Quaternion quat;
  // 90 deg x axis angle
  quat.w() = 0.7071068;
  quat.x() = 0.7071068;
  quat.y() = 0;
  quat.z() = 0;

  Vector3 vec_a(1, 2, 3);
  Vector3 vec_b = quat * vec_a;

  EXPECT_NEAR(vec_b.x(), vec_a.x(), 10e-6);
  EXPECT_NEAR(vec_b.y(), -vec_a.z(), 10e-6);
  EXPECT_NEAR(vec_b.z(), vec_a.y(), 10e-6);
}

TEST(TestQuaternion, convertToRotation)
{
  Quaternion quat = Quaternion::Identity();
  // 90 deg x axis angle
  quat.w() = 0.7071068;
  quat.x() = 0.7071068;
  quat.y() = 0;
  quat.z() = 0;
  Rotation rot = quat.toRotationMatrix();

  EXPECT_NEAR(rot(0, 0), 1, 10e-6);
  EXPECT_NEAR(rot(2, 1), 1, 10e-6);
  EXPECT_NEAR(rot(1, 2), -1, 10e-6);
}

TEST(TestQuaternion, logger)
{
  Quaternion quat;
  // 90 deg x axis angle
  quat.w() = 0.7071068;
  quat.x() = 0.7071068;
  quat.y() = 0;
  quat.z() = 0;

  medrctlog::info("Quaternion: \n{}", quat.toRotationMatrix());
}
