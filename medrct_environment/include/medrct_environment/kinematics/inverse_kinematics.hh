#pragma once

#include <memory>
#include <string>
#include <vector>

#include <medrct_common/types.hh>

namespace medrct
{
namespace env
{
using IKSolution = std::vector<real_t>;
class InverseKinematics
{
public:
  using Ptr = std::shared_ptr<InverseKinematics>;
  using ConstPtr = std::shared_ptr<const InverseKinematics>;

  InverseKinematics() = default;
  virtual ~InverseKinematics() = default;

  /**
   * @brief Calculates joint solutions given a pose of the tip link.
   * @param tip_transform
   * @param joint_positions_seed Vector of seed joint angles (size must match
   * number of joints in kinematic object)
   * @return A vector of solutions, If empty it failed to find a solution
   * (including uninitialized)
   */
  virtual std::vector<IKSolution> computeIK(
      const Transform& tip_transform,
      const std::vector<real_t>& joint_positions_seed =
          std::vector<real_t>()) const = 0;

  /** @brief Get the robot base link name */
  virtual std::string getBaseLinkName() const = 0;

  virtual std::string getTipLinkName() const = 0;

  /**
   * @brief Get list of joint names for kinematic object
   * @return A vector of joint names, joint_list_
   */
  virtual std::vector<std::string> getActiveJointNames() const = 0;

  /**
   * @brief Get the inverse kinematics working frame
   * @details This is the frame of reference in the poses given to the
   * computeIK function should be defined
   */
  virtual std::string getWorkingFrame() const {}
};
} // namespace env
} // namespace medrct