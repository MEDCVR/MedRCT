#include <cmath>

#include <medrct/log.hh>
#include <medrct_env/description/joint.hh>
#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/description/link.hh>

#include <medrct_franka_env/franka_kinematics_solver.hh>

namespace medrct
{
namespace env
{

Chain CreateFrankaChain()
{
  KinematicsTree kin_tree("franka_kinematics_tree");
  kin_tree.addLink(Link("base_link"));

  // 1st
  {
    kin_tree.addLink(Link("yaw_link"));
    Joint joint = Joint::FromModifiedDH(PI_2, 0, 0, PI_2, JointType::REVOLUTE);
    joint.setName("yaw_joint");
    joint.parent_link_name = "base_link";
    joint.child_link_name = "yaw_link";
    kin_tree.addJoint(joint);
  }
  // 2nd
  {
    kin_tree.addLink(Link("pitch_link"));
    Joint joint =
        Joint::FromModifiedDH(-PI_2, 0, 0, -PI_2, JointType::REVOLUTE);
    joint.setName("pitch_joint");
    joint.parent_link_name = "yaw_link";
    joint.child_link_name = "pitch_link";
    kin_tree.addJoint(joint);
  }
  // 3rd
  {
    kin_tree.addLink(Link("main_insertion_link"));
    Joint joint =
        Joint::FromModifiedDH(PI_2, 0, 0, -PI_2, JointType::PRISMATIC);
    joint.setName("main_insertion_joint");
    joint.parent_link_name = "pitch_link";
    joint.child_link_name = "main_insertion_link";
    kin_tree.addJoint(joint);
  }
  // 4th
  {
    kin_tree.addLink(Link("tool_roll_link"));
    Joint joint = Joint::FromModifiedDH(0, 0, 0, -PI_2, JointType::REVOLUTE);
    joint.setName("tool_roll_joint");
    joint.parent_link_name = "main_insertion_link";
    joint.child_link_name = "tool_roll_link";
    kin_tree.addJoint(joint);
  }
  // 5th
  {
    kin_tree.addLink(Link("tool_pitch_link"));
    Joint joint = Joint::FromModifiedDH(PI_2, 0, 0, PI_2, JointType::REVOLUTE);
    joint.setName("tool_pitch_joint");
    joint.parent_link_name = "tool_roll_link";
    joint.child_link_name = "tool_pitch_link";
    kin_tree.addJoint(joint);
  }
  // 6th
  {
    kin_tree.addLink(Link("tool_yaw_link"));
    Joint joint = Joint::FromModifiedDH(-PI_2, 0, 0, PI_2, JointType::REVOLUTE);
    joint.setName("tool_yaw_joint");
    joint.parent_link_name = "tool_pitch_link";
    joint.child_link_name = "tool_yaw_link";
    kin_tree.addJoint(joint);
  }
  // 6th->tool_tip
  {
    kin_tree.addLink(Link("tool_tip_link"));
    Joint joint = Joint::FromModifiedDH(PI_2, 0, 0, -PI_2, JointType::FIXED);
    joint.setName("tool_tip_joint");
    joint.parent_link_name = "tool_yaw_link";
    joint.child_link_name = "tool_tip_link";
    kin_tree.addJoint(joint);
  }
  // 6th->tool_gripper1_link
  {
    kin_tree.addLink(Link("tool_gripper1_link"));
    bool revert_axis = true;
    Joint joint = Joint::FromModifiedDH(
        PI_2, 0, 0, -PI_2, JointType::REVOLUTE, revert_axis);
    joint.setName("tool_gripper1_joint");
    joint.parent_link_name = "tool_yaw_link";
    joint.child_link_name = "tool_gripper1_link";
    kin_tree.addJoint(joint);
  }
  // 6th->tool_gripper2_link
  {
    kin_tree.addLink(Link("tool_gripper2_link"));
    Joint joint = Joint::FromModifiedDH(0, 0, 0, 0, JointType::REVOLUTE);
    joint.setName("tool_gripper2_joint");
    joint.parent_link_name = "tool_yaw_link";
    joint.child_link_name = "tool_gripper2_link";
    kin_tree.addJoint(joint);
  }
  Chain kin_chain;
  kin_chain.init(kin_tree, "base_link", "tool_tip_link");
}

FrankaForwardKinematics::FrankaForwardKinematics()
    : SimpleForwardKinematics(createFrankaChain())
{
}

FrankaInverseKinematics ::FrankaInverseKinematics()
{
}

inline real_t Sign(real_t val)
{
  return (0 < val) - (val < 0);
}
inline real_t GetAngleImpl(Vector3& vec_a, Vector3& vec_b)
{
  vec_a.normalize();
  vec_b.normalize();
  real_t vdot = vec_a.dot(vec_b);
  real_t angle;
  if ((1.0 - vdot) < 0.000001)
    angle = 0.0;
  else if ((1.0 + vdot) < 0.000001)
    angle = PI;
  else
    angle = std::acos(vdot);
  return angle;
}

inline real_t GetAngle(Vector3 vec_a, Vector3 vec_b)
{
  return GetAngleImpl(vec_a, vec_b);
}

inline real_t GetAngle(Vector3 vec_a, Vector3 vec_b, const Vector3& up_vector)
{
  real_t angle = GetAngleImpl(vec_a, vec_b);
  Vector3 cross_ab = vec_a.cross(vec_b);
  real_t same_dir = Sign(cross_ab.dot(up_vector));
  if (same_dir < 0.0)
    angle = -angle;
  return angle;
}

std::vector<IKSolution> FrankaInverseKinematics::computeIK(
    const Transform& tip_transform,
    const std::vector<real_t>& joint_positions_seed) const
{
  return {};
}
std::string FrankaInverseKinematics::getBaseLinkName() const
{
  return {"franka"};
}
std::string FrankaInverseKinematics::getTipLinkName() const
{
  return {"ee"};
}
std::vector<std::string> FrankaInverseKinematics::getActiveJointNames() const
{
  return {"links"};
}
} // namespace env
} // namespace medrct
