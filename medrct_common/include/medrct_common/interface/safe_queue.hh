#pragma once

#include "condition_wait.hh"

#include <condition_variable>
#include <mutex>
#include <queue>

namespace medrct
{
namespace stream
{

template <class T>
class SafeQueue
{
public:
  SafeQueue(unsigned int queue_size = 5) : queue_size(queue_size) {}
  ~SafeQueue() {}

  bool isEmpty() const
  {
    std::lock_guard<std::mutex> lock(mtx);
    return queue.empty();
  }
  T getLatest() const
  {
    std::lock_guard<std::mutex> lock(mtx);
    return queue.back();
  }
  void add(const T& data)
  {
    std::lock_guard<std::mutex> lock(mtx);
    queue.push(data);
    if (queue.size() > queue_size)
      queue.pop();
    notified = true;
    cv.notify_one();
  }
  bool waitForOne()
  {
    std::unique_lock<std::mutex> lock(mtx);
    notified = false;
    return ConditionWait::getInstance().wait(cv, lock, notified);
  }

private:
  unsigned int queue_size;
  mutable std::mutex mtx;
  std::condition_variable cv;
  bool notified;
  std::queue<T> queue;
};

} // namespace stream
} // namespace medrct
