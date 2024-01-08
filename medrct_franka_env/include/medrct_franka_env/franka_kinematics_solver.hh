#pragma once

#include <memory>
#include <vector>

#include <medrct/types/types.hh>
#include <medrct_env/description/chain.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>
#include <medrct_env/kinematics/simple_forward_kinematics.hh>

namespace medrct
{
namespace env
{

static Chain createFrankaChain();

class FrankaForwardKinematics : public SimpleForwardKinematics
{
public:
  FrankaForwardKinematics();
  virtual ~FrankaForwardKinematics() {}

private:
};

class FrankaInverseKinematics : public InverseKinematics
{
public:
  FrankaInverseKinematics();
  virtual ~FrankaInverseKinematics() {}
  std::vector<IKSolution> computeIK(
      const Transform& tip_transform,
      const std::vector<real_t>& joint_positions_seed =
          std::vector<real_t>()) const final;
  std::string getBaseLinkName() const final;
  std::string getTipLinkName() const final;
  std::vector<std::string> getActiveJointNames() const final;
};
} // namespace env
} // namespace medrct
