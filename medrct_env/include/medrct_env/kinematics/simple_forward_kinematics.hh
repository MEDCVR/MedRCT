#pragma once

#include <medrct_env/description/chain.hh>
#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>

namespace medrct
{
namespace env
{
class SimpleForwardKinematics : public ForwardKinematics
{
public:
  SimpleForwardKinematics(const Chain& chain);
  virtual ~SimpleForwardKinematics();
  virtual int computeFK(
      Transform& transform_out,
      const std::vector<real_t>& active_joint_positions,
      const unsigned int up_to_link_num = 1000) const override;
  virtual std::string getBaseLinkName() const override;
  virtual std::string getTipLinkName() const override;
  virtual std::vector<std::string> getLinkNames() const override;
  virtual std::vector<std::string> getActiveJointNames() const override;

private:
  const Chain kinematic_chain;
};
} // namespace env
} // namespace medrct
