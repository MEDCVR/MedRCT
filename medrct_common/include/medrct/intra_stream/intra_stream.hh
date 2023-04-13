#pragma once

#include <functional>
#include <memory>
#include <queue>
#include <thread>

#include "intra_common.hh"
#include "intra_executor.hh"
#include "medrct/stream/stream.hh"

namespace medrct
{
namespace stream
{

template <class dataT>
class IntraOutputStream : public OutputStream<dataT>
{
public:
  IntraOutputStream(const std::string& topic_name, const std::string& name)
      : OutputStream<dataT>(name)
  {
    StreamMaster& stream_master_ref = StreamMaster::getInstance();
    pub_connector_ptr =
        stream_master_ref.registerPublisher<dataT>(topic_name, deregister_func);
  }
  virtual ~IntraOutputStream() { deregister_func(); }

private:
  void publishImpl(const dataT& data) const final
  {
    pub_connector_ptr->publish(data);
  }
  std::shared_ptr<PublishConnector<dataT>> pub_connector_ptr;
  std::function<void()> deregister_func;
};

template <class dataT>
class IntraInputStream : public InputStream<dataT>, public ExecutionProcess
{
public:
  IntraInputStream(const std::string& topic_name, const std::string& name)
      : InputStream<dataT>(name)
  {
    StreamMaster& stream_master_ref = StreamMaster::getInstance();
    stream_master_ref.registerSubscriber<dataT>(
        topic_name,
        std::bind(
            &IntraInputStream::bufferDataToProcess,
            this,
            std::placeholders::_1),
        deregister_func);
  }
  virtual ~IntraInputStream() { deregister_func(); }

protected:
  void bufferDataToProcess(const dataT& data)
  {
    if (thread_notifier) // I don't like this implementation
    {
      std::unique_lock<std::mutex> executor_lock(thread_notifier->mtx);
      std::lock_guard<std::mutex> lock(data_mtx);
      data_to_proccess.push(data);
      thread_notifier->cv.notify_all();
    }
    else
    {
      std::lock_guard<std::mutex> lock(data_mtx);
      data_to_proccess.push(data);
    }
  }

  int getBufferSize() const final
  {
    std::lock_guard<std::mutex> lock(data_mtx);
    return data_to_proccess.size();
  }

  bool processOneData() final
  {
    std::lock_guard<std::mutex> lock(data_mtx);
    // notify executor if empty
    if (data_to_proccess.empty())
    {
      return false;
    }
    InputStream<dataT>::runCallback(data_to_proccess.front());
    data_to_proccess.pop();
    return true;
  }

private:
  std::function<void()> deregister_func;
  mutable std::mutex data_mtx;
  std::queue<dataT> data_to_proccess;
};
} // namespace stream
} // namespace medrct