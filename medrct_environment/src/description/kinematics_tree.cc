#include <algorithm>

#include <medrct_common/log.hh>
#include <medrct_environment/description/kinematics_tree.hh>

namespace medrct
{
namespace env
{
KinematicsTree::KinematicsTree(const std::string& name) : name(name)
{
}
KinematicsTree::KinematicsTree(KinematicsTree&& other) noexcept
    : name(std::move(other.name)),
      link_map(std::move(other.link_map)),
      joint_map(std::move(other.joint_map))
{
}
KinematicsTree& KinematicsTree::operator=(KinematicsTree&& other) noexcept
{
  name = std::move(other.name);
  link_map = std::move(other.link_map);
  joint_map = std::move(other.joint_map);
  return *this;
}

void KinematicsTree::setName(const std::string& name)
{
  this->name = name;
}

const std::string& KinematicsTree::getName() const
{
  return name;
}

const std::string& KinematicsTree::getRoot() const
{
  return tree_root_link_name;
}

bool KinematicsTree::addLink(const Link& link)
{
  if (link_map.find(link.getName()) != link_map.end())
  {
    // Link is already defined
    return false;
  }
  link_map[link.getName()] = std::make_shared<const Link>(link);
  if (link_map.size() == 1) // If first link added
  {
    tree_root_link_name = link.getName();
  }
  return true;
}

Link::ConstPtr KinematicsTree::getLink(const std::string& name) const
{
  auto it = link_map.find(name);
  if (it == link_map.end())
  {
    return nullptr;
  }
  return it->second;
}

std::vector<Link::ConstPtr> KinematicsTree::getLinks() const
{
  std::vector<Link::ConstPtr> link_vec;
  link_vec.reserve(link_map.size());
  for (auto it = link_map.begin(); it != link_map.end(); ++it)
  {
    link_vec.push_back(it->second);
  }
  return link_vec;
}

// TODO, there could be a link without a parent joint
bool KinematicsTree::addJoint(const Joint& joint)
{
  if (joint_map.find(joint.getName()) != joint_map.end())
  {
    // joint already exists
    medrctlog::error("Joint already exists");
    return false;
  }
  if (link_map.find(joint.parent_link_name) == link_map.end())
  {
    medrctlog::error("parent link doesnt exist");
    // parent link doesnt exist
    return false;
  }
  if (link_map.find(joint.child_link_name) == link_map.end())
  {
    // child link doesnt exist
    medrctlog::error("child link doesnt exist");
    return false;
  }

  if (link_to_parent_link.find(joint.child_link_name) !=
      link_to_parent_link.end())
  {
    medrctlog::error(
        "a link cannot have two parents!, this is a closed loop chain");
    // a link cannot have two parents!, this is a closed loop chain
    return false;
  }
  joint_map[joint.getName()] = std::make_shared<const Joint>(joint);
  link_to_parent_link[joint.child_link_name] = joint.parent_link_name;
  link_to_parent_joint[joint.child_link_name] = joint.getName();
  return true;
}

Joint::ConstPtr KinematicsTree::getJoint(const std::string& name) const
{
  auto it = joint_map.find(name);
  if (it == joint_map.end())
  {
    // joint doesn't exist
    return nullptr;
  }
  return it->second;
}

std::vector<Joint::ConstPtr> KinematicsTree::getJoints() const
{
  std::vector<Joint::ConstPtr> joint_vec;
  joint_vec.reserve(joint_map.size());
  for (auto it = joint_map.begin(); it != joint_map.end(); ++it)
  {
    joint_vec.push_back(it->second);
  }
  return joint_vec;
}

std::vector<Joint::ConstPtr> KinematicsTree::getActiveJoints() const
{
  std::vector<Joint::ConstPtr> joint_vec;
  joint_vec.reserve(joint_map.size());
  for (auto it = joint_map.begin(); it != joint_map.end(); ++it)
  {
    if (it->second->type != JointType::FIXED)
    {
      joint_vec.push_back(it->second);
    }
  }
  return joint_vec;
}

bool KinematicsTree::setRoot(const std::string& link_name)
{
  if (link_map.find(link_name) == link_map.end())
  {
    medrctlog::info("no link world");
    return false;
  }
  medrctlog::info(link_name);
  tree_root_link_name = link_name;
  return true;
}

bool KinematicsTree::getChainDescription(
    std::vector<std::string>& chain_joint_names,
    const std::string& root_link_name,
    const std::string& tip_link_name) const
{
  if (root_link_name == tip_link_name)
  {
    return false;
  }
  if (link_map.find(root_link_name) == link_map.end())
  {
    // root link doesn't exist
    return false;
  }
  if (link_map.find(tip_link_name) == link_map.end())
  {
    // tip link doesn't exist
    return false;
  }

  std::vector<std::string> reverse_joint_chain_names;
  reverse_joint_chain_names.reserve(joint_map.size());
  chain_joint_names.clear();
  std::string current_child_link = tip_link_name;
  while (current_child_link != tree_root_link_name)
  {
    reverse_joint_chain_names.push_back(
        link_to_parent_joint.at(current_child_link));
    current_child_link = link_to_parent_link.at(current_child_link);
    if (current_child_link == root_link_name)
    {
      std::reverse(
          reverse_joint_chain_names.begin(), reverse_joint_chain_names.end());
      chain_joint_names = reverse_joint_chain_names;
      return true;
    }
  }
  // root link is not in the same chain of tip link
  return false;
}
} // namespace env
} // namespace medrct
