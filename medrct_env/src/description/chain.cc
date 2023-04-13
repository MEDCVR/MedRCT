#include <medrct_env/description/chain.hh>

namespace medrct
{
namespace env
{
bool Chain::init(
    const KinematicsTree& kin_tree,
    const std::string& root_name,
    const std::string& tip_name)
{
  std::vector<std::string> joint_names;
  if (!kin_tree.getChainDescription(joint_names, root_name, tip_name))
  {
    return false;
  }
  unsigned int joint_names_size = joint_names.size();
  root_to_tip_joints.reserve(joint_names_size);
  link_names.reserve(joint_names_size + 1);
  for (unsigned int i = 0; i < joint_names_size; ++i)
  {
    const std::string& joint_name = joint_names[i];
    Joint::ConstPtr const_joint_ptr = kin_tree.getJoint(joint_name);
    if (i == 0)
    {
      link_names.push_back(const_joint_ptr->parent_link_name);
    }
    link_names.push_back(const_joint_ptr->child_link_name);
    if (const_joint_ptr->type != JointType::FIXED)
    {
      active_joint_names.push_back(joint_name);
    }
    root_to_tip_joints.push_back(const_joint_ptr);
  }
  return true;
}
const std::vector<Joint::ConstPtr>& Chain::getRootToTipJoints() const
{
  return root_to_tip_joints;
}
const std::vector<std::string>& Chain::getLinkNames() const
{
  return link_names;
}
const std::vector<std::string>& Chain::getActiveJointNames() const
{
  return active_joint_names;
}
} // namespace env
} // namespace medrct
