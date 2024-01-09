#pragma once

#include <memory>
#include <vector>

#include <medrct/types/types.hh>
#include <medrct_env/description/chain.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>
#include <medrct_env/kinematics/simple_forward_kinematics.hh>

#include <medrct_franka_control/franka_kinematics.hpp>

namespace medrct
{
namespace env
{

class FrankaForwardKinematics : public SimpleForwardKinematics
{
public:
  FrankaForwardKinematics();
  virtual ~FrankaForwardKinematics() {}
};

class FrankaInverseKinematics : public InverseKinematics
{
public:
  FrankaInverseKinematics();
  virtual ~FrankaInverseKinematics() {}
  std::vector<IKSolution> computeIK(
      const Transform& tip_transform,
      const std::vector<real_t>& joint_positions_seed =
          std::vector<real_t>()) final;
  std::string getBaseLinkName() const final;
  std::string getTipLinkName() const final;
  std::vector<std::string> getActiveJointNames() const final;

private:
  FrankaKinematicsSolver franka_kin_solver;
  Chain franka_chain;
};
} // namespace env
} // namespace medrct
