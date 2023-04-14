#pragma once

#include <vector>

#include <medrct/types/joint_state.hh>
#include <medrct/types/types.hh>
#include <medrct_env/description/kinematics_tree.hh>

#include "constraints.hh"

namespace medrct
{
namespace constrained
{
class Solver
{
public:
  Solver(const env::KinematicsTree::Ptr& kin_tree);
  ~Solver();
  bool addConstraint(const Constraint& constraint);
  bool solve(JointState& q, const Transform& tf, const JointState& q0) const;
  std::vector<Constraint> getConstraints() const;

private:
  env::KinematicsTree::Ptr kin_tree;
  std::vector<Constraint> constraints;
};
} // namespace constrained
} // namespace medrct
