#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "precision.hh"

namespace medrct
{
struct JointState
{
  std::vector<std::string> names;
  std::vector<real_t> positions;
  std::vector<real_t> velocities;
  std::vector<real_t> efforts;
  JointState() {}
  inline void reserve(int num)
  {
    names.reserve(num);
    positions.reserve(num);
    velocities.reserve(num);
    efforts.reserve(num);
  }
  inline void push_back(const std::string& name, real_t position)
  {
    names.push_back(name);
    positions.push_back(position);
    velocities.push_back(0);
    efforts.push_back(0);
  }
  friend std::ostream& operator<<(std::ostream& os, const JointState& js)
  {
    os << "names: \n";
    for (auto& n : js.names)
    {
      os << n << " ";
    }
    os << "\n";
    os << "positions: \n";
    for (auto& n : js.positions)
    {
      os << n << " ";
    }
    return os;
  }
};
} // namespace medrct
