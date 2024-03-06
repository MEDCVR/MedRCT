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
  kin_tree.addLink(Link("panda_link0"));

  {
    kin_tree.addLink(Link("panda_link1"));
    Joint joint = Joint::FromModifiedDH(0, 0, 0.333, 0, JointType::REVOLUTE);
    joint.setName("panda_joint1");
    joint.parent_link_name = "panda_link0";
    joint.child_link_name = "panda_link1";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link2"));
    Joint joint = Joint::FromModifiedDH(-PI_2, 0, 0, 0, JointType::REVOLUTE);
    joint.setName("panda_joint2");
    joint.parent_link_name = "panda_link1";
    joint.child_link_name = "panda_link2";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link3"));
    Joint joint = Joint::FromModifiedDH(PI_2, 0, 0.316, 0, JointType::REVOLUTE);
    joint.setName("panda_joint3");
    joint.parent_link_name = "panda_link2";
    joint.child_link_name = "panda_link3";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link4"));
    Joint joint =
        Joint::FromModifiedDH(PI_2, 0.0825, 0, 0, JointType::REVOLUTE);
    joint.setName("panda_joint4");
    joint.parent_link_name = "panda_link3";
    joint.child_link_name = "panda_link4";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link5"));
    Joint joint =
        Joint::FromModifiedDH(-PI_2, -0.0825, 0.384, 0, JointType::REVOLUTE);
    joint.setName("tool_pitch_joint");
    joint.parent_link_name = "panda_link4";
    joint.child_link_name = "panda_link5";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link6"));
    Joint joint = Joint::FromModifiedDH(PI_2, 0, 0, 0, JointType::REVOLUTE);
    joint.setName("panda_joint6");
    joint.parent_link_name = "panda_link5";
    joint.child_link_name = "panda_link6";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link7"));
    Joint joint = Joint::FromModifiedDH(PI_2, 0.088, 0, 0, JointType::FIXED);
    joint.setName("panda_joint7");
    joint.parent_link_name = "panda_link6";
    joint.child_link_name = "panda_link7";
    kin_tree.addJoint(joint);
  }
  {
    kin_tree.addLink(Link("panda_link8"));
    Joint joint = Joint::FromModifiedDH(0, 0, 0.107, 0, JointType::REVOLUTE);
    joint.setName("panda_joint8");
    joint.parent_link_name = "panda_link7";
    joint.child_link_name = "panda_link8";
    kin_tree.addJoint(joint);
  }
  Chain kin_chain;
  kin_chain.init(kin_tree, "panda_link0", "panda_link8");
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
