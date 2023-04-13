#pragma once

#include <memory>
#include <string>
#include <vector>

#include <medrct/types/types.hh>

namespace medrct
{
namespace env
{
/**
 *  @brief This class API assumes to work on a kinematic Chain
 */
class ForwardKinematics
{
public:
  using Ptr = std::shared_ptr<ForwardKinematics>;
  using ConstPtr = std::shared_ptr<const ForwardKinematics>;
  ForwardKinematics() = default;
  virtual ~ForwardKinematics() = default;

  /**
   * @brief This base class API is used for general things in
   * medrct_controller Calculates the transform from getBaseLinkName() to
   * up_to_link_num
   * @param transform_out The result transform
   * @param active_joint_positions The input joint positions to compute the
   * FK, size must match getActiveJointNames().size()
   * @param up_to_link_num If -1, will calculate up to getTipLinkName()
   * @return An error code if unsuccessful, 0 if success.
   */
  virtual int computeFK(
      Transform& transform_out,
      const std::vector<real_t>& active_joint_positions,
      const unsigned int up_to_link_num = 1000) const = 0;

  /** @brief Get the chain base link name */
  virtual std::string getBaseLinkName() const = 0;

  /** @brief Get the chain tip link name */
  virtual std::string getTipLinkName() const = 0;

  /** @brief Get all the link names. The size of this is the valid
   * up_to_link_num */
  virtual std::vector<std::string> getLinkNames() const = 0;
  /**
   * @brief Get list of active joint names for kinematic object
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getActiveJointNames() const = 0;
};
} // namespace env
} // namespace medrct
