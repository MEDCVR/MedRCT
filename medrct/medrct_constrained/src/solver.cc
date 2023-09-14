#include <medrct_constrained/solver.hh>

namespace medrct
{
namespace constrained
{
Solver::Solver(const env::KinematicsTree::Ptr& kin_tree) : kin_tree(kin_tree)
{
}
Solver::~Solver()
{
}
bool Solver::addConstraint(const Constraint& constraint)
{
  return true;
}
bool Solver::solve(
    JointState& q, const Transform& tf, const JointState& q0) const
{
  return true;
}
std::vector<Constraint> Solver::getConstraints() const
{
  return constraints;
}
} // namespace constrained
} // namespace medrct
