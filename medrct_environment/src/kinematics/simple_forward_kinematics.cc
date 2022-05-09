#include <medrct_common/log.hh>
#include <medrct_environment/description/joint.hh>
#include <medrct_environment/kinematics/simple_forward_kinematics.hh>

namespace medrct
{
namespace env
{
SimpleForwardKinematics::SimpleForwardKinematics(const Chain& chain)
    : kinematic_chain(chain)
{
}
SimpleForwardKinematics::~SimpleForwardKinematics()
{
}

int SimpleForwardKinematics::computeFK(
    Transform& transform_out,
    const std::vector<real_t>& active_joint_positions,
    const unsigned int up_to_link_num) const
{
  transform_out = Transform::Identity();
  unsigned int active_jp_idx = 0;
  unsigned int active_jp_size = active_joint_positions.size();
  for (unsigned int i = 0; i < kinematic_chain.getRootToTipJoints().size(); ++i)
  {
    if (i == up_to_link_num)
    {
      break;
    }
    Joint::ConstPtr joint_ptr = kinematic_chain.getRootToTipJoints()[i];
    if (joint_ptr->type == JointType::FIXED)
    {
      transform_out =
          transform_out * joint_ptr->parent_to_joint_origin_transform;
    }
    else // Is REVOLUTE/CONTINUOUS/PRISMATIC
    {
      if (active_jp_idx == active_jp_size)
      {
        return -1; // TODO error message
      }
      transform_out =
          transform_out * joint_ptr->getActuatedTransform(
                              active_joint_positions[active_jp_idx]);
      active_jp_idx++;
    }
  }
  return 0;
}

std::string SimpleForwardKinematics::getBaseLinkName() const
{
  return kinematic_chain.getRootToTipJoints().front()->parent_link_name;
}
std::string SimpleForwardKinematics::getTipLinkName() const
{
  return kinematic_chain.getRootToTipJoints().back()->child_link_name;
}
std::vector<std::string> SimpleForwardKinematics::getLinkNames() const
{
  return kinematic_chain.getLinkNames();
}
std::vector<std::string> SimpleForwardKinematics::getActiveJointNames() const
{
  return kinematic_chain.getActiveJointNames();
}
} // namespace env
} // namespace medrct
