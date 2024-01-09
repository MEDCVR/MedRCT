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
  return kin_chain;
}

FrankaForwardKinematics::FrankaForwardKinematics()
    : SimpleForwardKinematics(CreateFrankaChain())
{
}

FrankaInverseKinematics ::FrankaInverseKinematics()
    : franka_kin_solver(FrankaKinematicsSolver()),
      franka_chain(CreateFrankaChain())
{
}

std::vector<IKSolution> FrankaInverseKinematics::computeIK(
    const Transform& tip_transform,
    const std::vector<real_t>& joint_positions_seed)
{
  std::array<double, 7> solution = {joint_positions_seed[0],
                                    joint_positions_seed[1],
                                    joint_positions_seed[2],
                                    joint_positions_seed[3],
                                    joint_positions_seed[4],
                                    joint_positions_seed[5],
                                    joint_positions_seed[6]};
  if (!franka_kin_solver.computeIk(
          tip_transform.translation(),
          Eigen::Quaterniond(tip_transform.linear()),
          solution))
  {
    return {joint_positions_seed};
  }
  std::vector<real_t> ik_solution(solution.begin(), solution.end());
  return {ik_solution};
}

std::string FrankaInverseKinematics::getBaseLinkName() const
{
  return franka_chain.getRootToTipJoints().front()->parent_link_name;
}
std::string FrankaInverseKinematics::getTipLinkName() const
{
  return franka_chain.getRootToTipJoints().back()->child_link_name;
}
std::vector<std::string> FrankaInverseKinematics::getActiveJointNames() const
{
  return franka_chain.getActiveJointNames();
}

} // namespace env
} // namespace medrct
