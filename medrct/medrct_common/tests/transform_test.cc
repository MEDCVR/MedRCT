#include <gtest/gtest.h>

#include <medrct/types/types.hh>
#include <medrct/log.hh>

using namespace medrct;

TEST(TestTransform, constructor)
{
  Transform t = Transform::Identity();
  t.translation().x() = 1.0;
  EXPECT_EQ(t.translation().x(), 1.0);
  EXPECT_EQ(t.translation().y(), 0.0);
  EXPECT_EQ(t.translation().z(), 0.0);
  Quaternion q(t.linear());
  EXPECT_EQ(q.w(), 1.0);
  EXPECT_EQ(q.x(), 0.0);
  EXPECT_EQ(q.y(), 0.0);
  EXPECT_EQ(q.z(), 0.0);
}

TEST(TestTransform, multiplicationOperator)
{
  Transform t;
  t.translation().x() = 1.0;
  // 90 deg x axis angle
  Quaternion q;
  q.w() = 0.7071068;
  q.x() = 0.7071068;
  q.y() = 0;
  q.z() = 0;
  t.linear() = q.toRotationMatrix();
  Vector3 vec(1.0, 2.0, 3.0);
  Vector3 new_vec = t * vec;

  EXPECT_NEAR(new_vec.x(), 2.0, 10e-6);
  EXPECT_NEAR(new_vec.y(), -3.0, 10e-6);
  EXPECT_NEAR(new_vec.z(), 2.0, 10e-6);
}

TEST(TestTransform, logger)
{
  Transform t;
  t.translation().x() = 1.0;
  Quaternion q;
  q.w() = 0.7071068;
  q.x() = 0.7071068;
  q.y() = 0;
  q.z() = 0;
  t.linear() = q.toRotationMatrix();

  medrctlog::info("Transform Translation: \n{}", t.translation());
  medrctlog::info("Transform Rotation: \n{}", t.rotation());
}
