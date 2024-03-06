#pragma once

#include "types.hh"

namespace medrct
{
struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct Wrench
{
  Vector3 force;
  Vector3 torque;
};
} // namespace medrct
