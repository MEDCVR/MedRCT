#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

namespace medrct
{
namespace stream
{
class ConditionWait
{
private:
  ConditionWait();
  ~ConditionWait() = default;
  ConditionWait(const ConditionWait&) = delete;
  ConditionWait& operator=(const ConditionWait&) = delete;

  std::mutex mtx;
  std::function<bool()> is_running_func;

public:
  static ConditionWait& getInstance();
  void setIsRunningFunction(const std::function<bool()>& is_running_func);
  bool wait(
      std::condition_variable& cv,
      std::unique_lock<std::mutex>& lock,
      bool& notified);
};
} // namespace stream
} // namespace medrct
