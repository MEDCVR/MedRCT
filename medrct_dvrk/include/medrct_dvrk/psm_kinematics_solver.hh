#pragma once

#include <memory>
#include <vector>

#include <medrct_common/types.hh>
#include <medrct_environment/description/chain.hh>
#include <medrct_environment/kinematics/inverse_kinematics.hh>
#include <medrct_environment/kinematics/simple_forward_kinematics.hh>

#include "psm_tool_params.hh"

namespace medrct
{
namespace env
{

class PsmKinematicsData
{
public:
  PsmKinematicsData() = default;
  ~PsmKinematicsData() = default;
  PsmKinematicsData(const PsmKinematicsData& other);
  PsmKinematicsData& operator=(const PsmKinematicsData& other);

  void init(const SphericalWristToolParams& swt_params);
  const Chain& getChain() const;
  const SphericalWristToolParams& getParams() const;

private:
  Chain chain;
  std::unique_ptr<SphericalWristToolParams> swt_params;
};

class PsmForwardKinematics : public SimpleForwardKinematics
{
public:
  PsmForwardKinematics(const PsmKinematicsData& psm_kin_data);
  virtual ~PsmForwardKinematics();
};

class PsmInverseKinematics : public InverseKinematics
{
public:
  PsmInverseKinematics(const PsmKinematicsData& psm_kin_data);
  virtual ~PsmInverseKinematics();
  std::vector<IKSolution> computeIK(
      const Transform& tip_transform,
      const std::vector<real_t>& joint_positions_seed =
          std::vector<real_t>()) const final;
  std::string getBaseLinkName() const final;
  std::string getTipLinkName() const final;
  std::vector<std::string> getActiveJointNames() const final;

private:
  const PsmKinematicsData psm_kin_data;
  const PsmForwardKinematics psm_fwd_kin;
};
} // namespace env
} // namespace medrct
