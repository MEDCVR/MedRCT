#include <assert.h>

#include <string>
#include <vector>

#include <medrct/types/types.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>
#include <medrct_env/kinematics/inverse_kinematics.hh>

using namespace medrct;
using namespace medrct::stream;

class XPrismaticForwardKinematics : public env::ForwardKinematics
{
public:
  XPrismaticForwardKinematics() {}
  virtual ~XPrismaticForwardKinematics() {}
  virtual int computeFK(
      Transform& transform_out,
      const std::vector<real_t>& active_joint_positions,
      const unsigned int) final
  {
    assert((active_joint_positions.size() == 1));
    transform_out.translation().x() = active_joint_positions[0];
    return 0;
  }
  std::string getBaseLinkName() const final { return "base_link"; }
  std::string getTipLinkName() const final { return "tip_link"; }

  std::vector<std::string> getLinkNames() const final
  {
    return {"base_link", "tip_link"};
  }

  std::vector<std::string> getActiveJointNames() const final
  {
    return {"base_tip_joint"};
  };
};

class XPrismaticInverseKinematics : public env::InverseKinematics
{
public:
  XPrismaticInverseKinematics() {}
  virtual ~XPrismaticInverseKinematics() {}

  std::vector<env::IKSolution>
  computeIK(const Transform& tip_transform, const std::vector<real_t>&) final
  {
    std::vector<real_t> positions = {tip_transform.translation().x()};
    return {positions};
  }
  std::string getBaseLinkName() const final { return "base_link"; }

  std::string getTipLinkName() const final { return "tip_link"; }

  std::vector<std::string> getActiveJointNames() const final
  {
    return {"base_tip_joint"};
  };
};
