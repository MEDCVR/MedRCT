#pragma once

#include <medrct/types/types.hh>

namespace medrct
{
namespace env
{
struct SphericalWristToolParams
{
  using Ptr = std::shared_ptr<SphericalWristToolParams>;
  using ConstPtr = std::shared_ptr<const SphericalWristToolParams>;
  SphericalWristToolParams(
      real_t l_rcc,
      real_t l_tool,
      real_t l_pitch2yaw,
      real_t l_yaw2ctrlpnt,
      real_t scale = 1.0)
      : l_rcc(l_rcc),
        l_tool(l_tool),
        l_pitch2yaw(l_pitch2yaw),
        l_yaw2ctrlpnt(l_yaw2ctrlpnt),
        l_tool2rcm_offset(l_rcc - l_tool),
        scale(scale)
  {
  }
  real_t l_rcc, l_tool, l_pitch2yaw;
  real_t l_yaw2ctrlpnt, l_tool2rcm_offset, scale;
};

struct LND400006 : SphericalWristToolParams
{
  LND400006(real_t scale = 1.0)
      : SphericalWristToolParams(0.4318, 0.4162, 0.0091, 0.0102, scale)
  {
  }
};
} // namespace env
} // namespace medrct
