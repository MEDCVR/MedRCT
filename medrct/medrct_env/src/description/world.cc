#include <medrct_env/description/link.hh>
#include <medrct_env/description/joint.hh>
#include <medrct_env/description/world.hh>

#include <urdf_parser/urdf_parser.h>
namespace medrct
{
namespace env
{
World::World()
{
}
World::~World()
{
}
World& World::getInstance()
{
  static World w;
  return w;
}
void World::parseURDFFile(const std::string& urdf_file_path)
{
  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDFFile(urdf_file_path);
  if (!urdf)
  {
    throw std::runtime_error("World failed to parse URDF File");
  }
  // Everything should be world as root right now.
  if (urdf->links_.find("world") == urdf->links_.end())
  {
    throw std::runtime_error("URDF must include a [world] link");
  }
  std::string model_name = urdf->getName();
  if (name_to_kinematic_trees.find(model_name) != name_to_kinematic_trees.end())
  {
    throw std::runtime_error(
        "URDF with robot name [" + model_name + "] already exists!");
  }
  KinematicsTree::Ptr kin_tree = std::make_shared<KinematicsTree>(model_name);
  for (auto it = urdf->links_.begin(); it != urdf->links_.end(); it++)
  {
    const auto& urdf_link = it->second;
    Link link(urdf_link->name);
    // TODO, visualization and collision here
    kin_tree->addLink(link);
  }
  for (auto it = urdf->joints_.begin(); it != urdf->joints_.end(); it++)
  {
    const auto& urdf_joint = it->second;
    Joint joint(urdf_joint->name);
    joint.child_link_name = urdf_joint->child_link_name;
    joint.parent_link_name = urdf_joint->parent_link_name;
    if (urdf_joint->type == urdf::Joint::FIXED)
      joint.type = JointType::FIXED;
    else if (urdf_joint->type == urdf::Joint::REVOLUTE)
      joint.type = JointType::REVOLUTE;
    else if (urdf_joint->type == urdf::Joint::CONTINUOUS)
      joint.type = JointType::CONTINUOUS;
    else if (urdf_joint->type == urdf::Joint::PRISMATIC)
      joint.type = JointType::PRISMATIC;
    else
      throw std::runtime_error(
          "URDF unsupported joint type for [" + urdf_joint->name + "] joint.");
    joint.axis.x() = urdf_joint->axis.x;
    joint.axis.y() = urdf_joint->axis.y;
    joint.axis.z() = urdf_joint->axis.z;
    // TODO add limits, mimic
    if (!kin_tree->addJoint(joint))
    {
      // TODO better error message throw from Kinematics Tree
      throw std::runtime_error(
          "URDF kinematics tree failed to add [" + urdf_joint->name +
          "] joint.");
    }
  }
  kin_tree->setRoot("world");
  // TODO check if kin_tree is valid parent and child all connected
  name_to_kinematic_trees[model_name] = kin_tree;
}

KinematicsTree::Ptr World::getKinematicsTree(const std::string& name)
{
  auto it = name_to_kinematic_trees.find(name);
  if (it == name_to_kinematic_trees.end())
  {
    return nullptr;
  }
  return it->second;
}
} // namespace env
} // namespace medrct
