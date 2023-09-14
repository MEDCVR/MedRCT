#include <cmath>

#include <medrct/log.hh>
#include <medrct_env/description/joint.hh>
#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/description/link.hh>

#include <medrct_dvrk_env/psm_kinematics_solver.hh>

namespace medrct
{
namespace env
{

PsmKinematicsData::PsmKinematicsData(const PsmKinematicsData& other)
{
  this->chain = other.chain;
  this->swt_params =
      std::make_unique<SphericalWristToolParams>(*other.swt_params);
}
PsmKinematicsData& PsmKinematicsData::operator=(const PsmKinematicsData& other)
{
  this->chain = other.chain;
  this->swt_params =
      std::make_unique<SphericalWristToolParams>(*other.swt_params);
  return *this;
}

void PsmKinematicsData::init(const SphericalWristToolParams& swt_params)
{
  KinematicsTree kin_tree("psm_kinematics_tree");
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
    Joint joint = Joint::FromModifiedDH(
        PI_2, 0, -swt_params.l_rcc, -PI_2, JointType::PRISMATIC);
    joint.setName("main_insertion_joint");
    joint.parent_link_name = "pitch_link";
    joint.child_link_name = "main_insertion_link";
    kin_tree.addJoint(joint);
  }
  // 4th
  {
    kin_tree.addLink(Link("tool_roll_link"));
    Joint joint = Joint::FromModifiedDH(
        0, 0, swt_params.l_tool, -PI_2, JointType::REVOLUTE);
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
    Joint joint = Joint::FromModifiedDH(
        -PI_2, swt_params.l_pitch2yaw, 0, PI_2, JointType::REVOLUTE);
    joint.setName("tool_yaw_joint");
    joint.parent_link_name = "tool_pitch_link";
    joint.child_link_name = "tool_yaw_link";
    kin_tree.addJoint(joint);
  }
  // 6th->tool_tip
  {
    kin_tree.addLink(Link("tool_tip_link"));
    Joint joint = Joint::FromModifiedDH(
        PI_2, 0, swt_params.l_yaw2ctrlpnt, -PI_2, JointType::FIXED);
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
        PI_2,
        0,
        swt_params.l_yaw2ctrlpnt,
        -PI_2,
        JointType::REVOLUTE,
        revert_axis);
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
  this->chain = std::move(kin_chain);
  this->swt_params =
      std::make_unique<SphericalWristToolParams>(std::move(swt_params));
  return;
}

const Chain& PsmKinematicsData::getChain() const
{
  return chain;
}
const SphericalWristToolParams& PsmKinematicsData::getParams() const
{
  return *swt_params;
}

PsmForwardKinematics::PsmForwardKinematics(
    const PsmKinematicsData& psm_kin_data)
    : SimpleForwardKinematics(psm_kin_data.getChain())
{
}
PsmForwardKinematics::~PsmForwardKinematics()
{
}

PsmInverseKinematics::PsmInverseKinematics(
    const PsmKinematicsData& psm_kin_data)
    : psm_kin_data(psm_kin_data),
      psm_fwd_kin(PsmForwardKinematics(psm_kin_data))
{
}
PsmInverseKinematics::~PsmInverseKinematics()
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

std::vector<IKSolution> PsmInverseKinematics::computeIK(
    const Transform& tip_transform, const std::vector<real_t>&) const
{
  Transform T_pinchjoint_7 = Transform::Identity();
  T_pinchjoint_7.translation() =
      Vector3::UnitZ() * -1 * psm_kin_data.getParams().l_yaw2ctrlpnt;
  Transform T_pinchjoint_0 = tip_transform * T_pinchjoint_7;

  Vector3 n_palmjoint_pinchjoint =
      T_pinchjoint_0.linear().inverse() * T_pinchjoint_0.translation() * -1;
  n_palmjoint_pinchjoint.x() = 0;
  n_palmjoint_pinchjoint.normalize();

  Transform T_palmjoint_pinchjoint = Transform::Identity();
  T_palmjoint_pinchjoint.translation() =
      n_palmjoint_pinchjoint * psm_kin_data.getParams().l_pitch2yaw;

  Transform T_palmjoint_0 =
      tip_transform * T_pinchjoint_7 * T_palmjoint_pinchjoint;
  real_t xz_diagonal = std::sqrt(
      std::pow(T_palmjoint_0.translation().x(), 2) +
      std::pow(T_palmjoint_0.translation().z(), 2));
  real_t j1 = std::atan2(
      T_palmjoint_0.translation().x(), -T_palmjoint_0.translation().z());
  real_t j2 = -std::atan2(T_palmjoint_0.translation().y(), xz_diagonal);
  real_t j3 = T_palmjoint_0.translation().norm() +
              psm_kin_data.getParams().l_tool2rcm_offset;

  Vector3 cross_palmlink_x7_0 = tip_transform.linear().col(0).cross(
      T_pinchjoint_0.translation() - T_palmjoint_0.translation());

  Transform T_3_0;
  psm_fwd_kin.computeFK(T_3_0, {j1, j2, j3}, 3);

  real_t j4 = GetAngle(
      cross_palmlink_x7_0,
      T_3_0.linear().col(0) * -1,
      T_3_0.linear().col(2) * -1);

  const std::vector<Joint::ConstPtr>& root_to_tip_joints =
      psm_kin_data.getChain().getRootToTipJoints();
  Transform T_4_0 = T_3_0 * root_to_tip_joints[3]->getActuatedTransform(j4);

  real_t j5 = GetAngle(
      T_pinchjoint_0.translation() - T_palmjoint_0.translation(),
      T_4_0.linear().col(2),
      T_4_0.linear().col(1));
  Transform T_5_0 = T_4_0 * root_to_tip_joints[4]->getActuatedTransform(j5);
  real_t j6 = GetAngle(
      tip_transform.linear().col(2),
      T_5_0.linear().col(0),
      T_5_0.linear().col(1) * -1);

  return {{j1, j2, j3, j4, j5, j6}};
}
std::string PsmInverseKinematics::getBaseLinkName() const
{
  return psm_kin_data.getChain().getLinkNames().front();
}
std::string PsmInverseKinematics::getTipLinkName() const
{
  return psm_kin_data.getChain().getLinkNames().back();
}
std::vector<std::string> PsmInverseKinematics::getActiveJointNames() const
{
  return psm_kin_data.getChain().getActiveJointNames();
}
} // namespace env
} // namespace medrct
