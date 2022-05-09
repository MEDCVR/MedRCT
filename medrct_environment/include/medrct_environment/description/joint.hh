#pragma once

#include <memory>
#include <string>

#include <medrct_common/types.hh>
#include <medrct_environment/joint_limit.hh>

namespace medrct
{
namespace env
{
enum class JointType
{
  UNKNOWN,
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  // FLOATING,
  // PLANAR,
  FIXED
};

class JointMimic
{
public:
  using Ptr = std::shared_ptr<JointMimic>;
  using ConstPtr = std::shared_ptr<const JointMimic>;

  JointMimic() = default;
  JointMimic(real_t offset, real_t multiplier, std::string joint_name)
      : offset(offset),
        multiplier(multiplier),
        joint_name(std::move(joint_name))
  {
  }
  real_t offset{0};
  real_t multiplier{1.0};
  std::string joint_name;

  void clear()
  {
    offset = 0.0;
    multiplier = 1.0;
    joint_name.clear();
  }

  friend std::ostream& operator<<(std::ostream& os, const JointMimic& mimic)
  {
    os << "joint_name=" << mimic.joint_name << " offset=" << mimic.offset
       << " multiplier=" << mimic.multiplier;
    return os;
  };
};

class Joint
{
public:
  using Ptr = std::shared_ptr<Joint>;
  using ConstPtr = std::shared_ptr<const Joint>;
  Joint(const std::string& name) : name(name) {}
  Joint() = default;
  ~Joint() = default;

  Joint(const Joint& other) = default;
  Joint& operator=(const Joint& other) = default;

  Joint(Joint&& other) = default;
  Joint& operator=(Joint&& other) = default;

  void setName(const std::string& name) { this->name = name; }
  const std::string& getName() const { return name; }

  /// The type of joint
  JointType type{JointType::UNKNOWN};

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  /// This should always be a unit vector.
  Vector3 axis;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;

  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;

  /// transform from Parent Link frame to Joint frame
  Transform parent_to_joint_origin_transform = Transform::Identity();

  /// Joint Limits
  JointLimit limits;

  /// Option to Mimic another Joint
  JointMimic::ConstPtr mimic;

  // If prismatic: meters
  // If revolute/continuous: radians
  // If fixed, will just get the parent_to_joint_origin_transform
  Transform getActuatedTransform(real_t joint_position) const;

  static Joint FromModifiedDH(
      real_t alpha,
      real_t a,
      real_t d,
      real_t theta,
      JointType joint_type,
      bool revert_axis = false);

private:
  std::string name;
};
} // namespace env
} // namespace medrct
