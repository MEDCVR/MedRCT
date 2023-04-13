#include <gtest/gtest.h>

#include <medrct/log.hh>
#include <medrct/types/types.hh>
#include <medrct_env/description/world.hh>
#include <urdf_parser/urdf_parser.h>

using namespace medrct::env;

TEST(TestWorld, testOutput)
{
  std::string joint_str =
      "<robot name=\"test\">"
      "  <joint name=\"s-j1\" type=\"fixed\">"
      "    <parent link=\"l1\"/>"
      "    <child link=\"l2\"/>"
      "  </joint>"
      "  <joint name=\"r-j2\" type=\"fixed\">"
      "    <parent link=\"l1\"/>"
      "    <child link=\"l2\"/>"
      "  </joint>"
      "  <link name=\"l1\">"
      "    <visual>"
      "      <geometry>"
      "        <sphere radius=\"1.349\"/>"
      "      </geometry>"
      "      <material name=\"\">"
      "        <color rgba=\"1.0 0.65 0.0 0.01\" />"
      "      </material>"
      "    </visual>"
      "    <inertial>"
      "      <mass value=\"8.4396\"/>"
      "      <inertia ixx=\"0.087\" ixy=\"0.14\" ixz=\"0.912\" iyy=\"0.763\" "
      "iyz=\"0.0012\" izz=\"0.908\"/>"
      "    </inertial>"
      "  </link>"
      "  <link name=\"l2\">"
      "    <visual>"
      "      <geometry>"
      "        <cylinder radius=\"3.349\" length=\"7.5490\"/>"
      "      </geometry>"
      "      <material name=\"red ish\">"
      "        <color rgba=\"1 0.0001 0.0 1\" />"
      "      </material>"
      "    </visual>"
      "  </link>"
      "</robot>";
  auto urdf = urdf::parseURDF(joint_str);
  medrctlog::info("num joints: {}", urdf->joints_.size());
  for (auto it = urdf->joints_.begin(); it != urdf->joints_.end(); it++)
  {
    medrctlog::info("name {}", it->first);
  }
  for (auto it = urdf->links_.begin(); it != urdf->links_.end(); it++)
  {
    medrctlog::info("name {}", it->first);
  }
}

TEST(TestWorld, testLoadFromFile)
{
  auto& world_inst = World::getInstance();
  // This is suppose to throw, if not correct
  medrctlog::info("urdf path: {}", std::string(TEST_WORLD_URDF_PATH));
  world_inst.parseURDFFile(std::string(TEST_WORLD_URDF_PATH));
  auto kin_tree_ptr = world_inst.getKinematicsTree("test_robot");
  ASSERT_TRUE(kin_tree_ptr != nullptr);

  ASSERT_EQ(kin_tree_ptr->getName(), "test_robot");
  ASSERT_EQ(kin_tree_ptr->getRoot(), "world");

  medrctlog::info(
      "CMAKE_CURRENT_LIST_DIR: {}", std::string(CMAKE_CURRENT_LIST_DIR));

  auto links = kin_tree_ptr->getLinks();
  ASSERT_EQ(links.size(), 3);
  auto joints = kin_tree_ptr->getJoints();
  ASSERT_EQ(joints.size(), 2);

  ASSERT_TRUE(kin_tree_ptr->getLink("world") != nullptr);
  ASSERT_TRUE(kin_tree_ptr->getLink("l1") != nullptr);
  ASSERT_TRUE(kin_tree_ptr->getLink("l2") != nullptr);
  ASSERT_TRUE(kin_tree_ptr->getJoint("world_l1_joint") != nullptr);
  ASSERT_TRUE(kin_tree_ptr->getJoint("l1_l2_joint") != nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
