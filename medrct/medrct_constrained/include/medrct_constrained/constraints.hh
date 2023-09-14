#pragma once

#include <medrct_env/description/object.hh>

namespace medrct
{
namespace constrained
{
struct Constraint
{
  enum class type_t
  {
    UNKNOWN = -1,
    ATTRACTOR = 1,
    BOUNDARY = 2,
  };
  type_t type;
  env::Object object;
};
} // namespace constrained
} // namespace medrct
