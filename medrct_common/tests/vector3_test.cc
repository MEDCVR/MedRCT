#include <gtest/gtest.h>

#include <medrct_common/types.hh>
#include <medrct_common/log.hh>

using namespace medrct;

// Declare a test
TEST(TestVector3D, constructor)
{
  Vector3 vector3d;
  vector3d.x() = 2.0;
  vector3d.y() = 3.0;
  vector3d.z() = 4.0;
  EXPECT_EQ(vector3d.x(), 2.0);
  EXPECT_EQ(vector3d.y(), 3.0);
  EXPECT_EQ(vector3d.z(), 4.0);
}

TEST(TestVector3D, addition)
{
  Vector3 a(1, 2, 3), b(2, 4, 6);
  Vector3 c = a + b;

  EXPECT_EQ(c.x(), 3);
  EXPECT_EQ(c.y(), 6);
  EXPECT_EQ(c.z(), 9);
}

TEST(TestVector3D, logger)
{
  Vector3 a(1, 2, 3);

  medrctlog::info("Vector: \n{}", a);
}
