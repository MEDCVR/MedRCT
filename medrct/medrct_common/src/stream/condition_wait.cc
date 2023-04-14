#include <medrct/stream/condition_wait.hh>

namespace medrct
{
namespace stream
{

ConditionWait::ConditionWait()
{
  is_running_func = []() { return true; };
}

ConditionWait& ConditionWait::getInstance()
{
  static ConditionWait w;
  return w;
}

void ConditionWait::setIsRunningFunction(
    const std::function<bool()>& is_running_func)
{
  std::lock_guard<std::mutex> lk(this->mtx);
  this->is_running_func = is_running_func;
}

bool ConditionWait::wait(
    std::condition_variable& cv,
    std::unique_lock<std::mutex>& lock,
    bool& notified)
{
  bool is_running;
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    is_running = is_running_func();
  }
  while (is_running)
  {
    cv.wait_for(lock, std::chrono::milliseconds(500));
    if (notified)
    {
      return true;
    }
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      is_running = is_running_func();
    }
  }
  return false;
}
} // namespace stream
} // namespace medrct
