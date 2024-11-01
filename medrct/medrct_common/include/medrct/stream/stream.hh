#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "safe_queue.hh"

#include <medrct/log.hh>

namespace medrct
{
namespace stream
{
enum class stream_type_t
{
  OUTPUT = 0,
  INPUT = 1,
  STREAM_END = 2
};

// TODO: Make the C++ shared memory version instantiation, and see if
// interfaces makes sense.

struct Stream
{
  using Ptr = std::shared_ptr<Stream>;
  using ConstPtr = std::shared_ptr<const Stream>;
  Stream(const stream_type_t stream_type, const std::string& name)
      : stream_type(stream_type), name(name)
  {
  }
  virtual ~Stream() = 0;
  const stream_type_t stream_type;
  const std::string name;
};

template <class T>
class PubStream : public Stream
{
public:
  using Ptr = std::shared_ptr<PubStream<T>>;
  using ConstPtr = std::shared_ptr<const PubStream<T>>;
  PubStream(const std::string& name) : Stream(stream_type_t::OUTPUT, name) {}
  virtual ~PubStream() {}
  void publish(const T& data) const { publishImpl(data); }

private:
  virtual void publishImpl(const T& data) const = 0;
};

class A
{
  A(){}
};

// BBase class example for config overloading?
class BBase : public A
{
};

template <class T>
class B : public BBase
{
  B(){}
};

template <class T>
class SubStream : public Stream
{
public:
  using Ptr = std::shared_ptr<SubStream<T>>;
  using ConstPtr = std::shared_ptr<const SubStream<T>>;
  SubStream(const std::string& name) : Stream(stream_type_t::INPUT, name) {}
  virtual ~SubStream() = 0;
  bool addCallback(
      const std::string& callback_name,
      const std::function<void(const T&)>& callback)
  {
    std::lock_guard<std::mutex> lock(callback_mtx);
    if (callback_map.find(callback_name) != callback_map.end())
    {
      return false;
    }
    callback_map[callback_name] = callback;
    return true;
  }
  bool removeCallback(const std::string& callback_name)
  {
    std::lock_guard<std::mutex> lock(callback_mtx);
    auto it = callback_map.find(callback_name);
    if (it == callback_map.end())
    {
      return false;
    }
    callback_map.erase(it);
    return true;
  }

protected:
  void runCallback(const T& data)
  {
    std::lock_guard<std::mutex> lock(callback_mtx);
    for (auto& callback : callback_map)
    {
      callback.second(data);
    }
    return;
  }

private:
  std::mutex callback_mtx;
  std::map<std::string, std::function<void(const T&)>> callback_map;

public:
  void addBuffer(int queue_size = 5)
  {
    if (!safe_queue)
    {
      safe_queue = std::make_unique<SafeQueue<T>>(queue_size);
    }
    addCallback(
        "buffer",
        std::bind(&SafeQueue<T>::add, safe_queue.get(), std::placeholders::_1));
    // Useful for debugging with name print
    // addCallback("buffer", [this](const T& data) {
    //   if (this->name == "Keyboard-PSM2_feedback_js_stream")
    //   {
    //     // medrctlog::info("Trying to add");
    //   }
    //   safe_queue->add(data);
    // });
    is_buffer_on.store(true);
  }
  void removeBuffer()
  {
    removeCallback("buffer");
    is_buffer_on.store(false);
  }
  const SafeQueue<T>& getBuffer() const
  {
    if (!is_buffer_on.load())
    {
      throw std::logic_error{"Buffer not added"};
    }
    return *safe_queue;
  }
  bool waitForOneData() const
  {
    if (is_buffer_on.load())
    {
      return safe_queue->waitForOne();
    }
    return false;
  }

private:
  std::atomic<bool> is_buffer_on{false};
  std::unique_ptr<SafeQueue<T>> safe_queue;
};

template <class T>
SubStream<T>::~SubStream()
{
}
} // namespace stream
} // namespace medrct
