#pragma once

#include <sstream>
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"

namespace medrctlog = spdlog;

template <typename T>
std::string TypeToString(T value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}
