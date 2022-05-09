#pragma once

#include <vector>

#include <medrct_common/types.hh>

namespace medrct
{
struct RangeVec
{
  std::vector<real_t> min;
  std::vector<real_t> max;
};

struct JointLimit
{
  RangeVec position_limits;
  RangeVec velocity_limits;
  RangeVec effort_limits;
};
} // namespace medrct
