#include <assert.h>
#include <cmath>

#include <medrct/log.hh>
#include <medrct_env/description/joint.hh>

namespace medrct
{
namespace env
{
Transform Joint::getActuatedTransform(real_t joint_position) const
{
  switch (type)
  {
    case JointType::FIXED:
      return parent_to_joint_origin_transform;
    case JointType::PRISMATIC:
    {
      Transform new_tf = parent_to_joint_origin_transform;
      new_tf.translation() =
          parent_to_joint_origin_transform.translation() +
          parent_to_joint_origin_transform.linear() * axis * joint_position;
      return new_tf;
    }
    default: // REVOLUTE OR CONTINUOUS
    {
      Transform new_tf = parent_to_joint_origin_transform;
      new_tf.linear() = parent_to_joint_origin_transform.linear() *
                        Quaternion(AngleAxis(joint_position, axis));
      return new_tf;
    }
  }
}

// new_joint but with missing, child_link_name, parent_link_name, limits, and
// mimic
Joint Joint::FromModifiedDH(
    real_t alpha,
    real_t a,
    real_t d,
    real_t theta,
    JointType joint_type,
    bool revert_axis)
{
  bool is_correct_type = joint_type == JointType::REVOLUTE ||
                         joint_type == JointType::CONTINUOUS ||
                         joint_type == JointType::PRISMATIC ||
                         joint_type == JointType::FIXED;
  assert(
      ("Only Revolute, Continuous, or Prismatic supported", is_correct_type));
  Joint new_joint;
  new_joint.type = joint_type;
  new_joint.axis = Vector3::UnitZ();
  if (revert_axis)
  {
    new_joint.axis = new_joint.axis * -1;
  }
  new_joint.parent_to_joint_origin_transform = Transform::Identity();

  real_t ca = std::cos(alpha);
  real_t sa = std::sin(alpha);
  real_t ct = std::cos(theta);
  real_t st = std::sin(theta);
  new_joint.parent_to_joint_origin_transform.translation() =
      Vector3(a, -d * sa, d * ca);
  new_joint.parent_to_joint_origin_transform.linear() =
      FromMatrix(ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca)
          .toRotationMatrix();

  return new_joint;
}
} // namespace env
} // namespace medrct
