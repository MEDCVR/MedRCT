#pragma once

#include <memory>
#include <string>
#include <vector>

#include <medrct/types/types.hh>
#include <medrct_env/kinematics_tree.hh>
#include "medrct_env/visualizer/vizualizer.hh"

namespace medrct
{
namespace env
{
class Robot
{
public:
  // vizualizer (optional)
  Robot(
      std::shared_ptr<KinematicsTree> kin_tree,
      std::shared_ptr<Vizualizer> vizualizer);
  ~Robot();
  void updateJointState(const JointState& js);
};
} // namespace env
} // namespace medrct
