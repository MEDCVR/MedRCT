#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <medrct_common/log.hh>

namespace medrct
{
namespace stream
{

struct Connector
{
  virtual ~Connector() = 0;
};

template <class dataT>
class PublishConnector : public Connector
{
public:
  PublishConnector() {}
  virtual ~PublishConnector() = 0;
  void publish(const dataT& data)
  {
    std::lock_guard<std::mutex> cb_lock(cb_mtx);
    for (auto it = callbacks.begin(); it != callbacks.end(); it++)
    {
      it->second(data);
    }
  }

protected:
  std::mutex cb_mtx;
  int count = 0;
  std::map<int, std::function<void(const dataT&)>> callbacks;
};

template <class dataT>
PublishConnector<dataT>::~PublishConnector()
{
}

template <class dataT>
class RegisterConnector : public PublishConnector<dataT>
{
public:
  RegisterConnector() {}
  virtual ~RegisterConnector() {}
  // These should be efficient callbacks just for storing data
  // Otherwise publish thread will be too blocked
  std::function<void()>
  registerCb(const std::function<void(const dataT&)>& data_store_callback)
  {
    std::lock_guard<std::mutex> lock(this->cb_mtx);
    int current_count = this->count;
    this->callbacks[current_count] = data_store_callback;
    auto deregisterer_func = [this, current_count]() {
      auto it = this->callbacks.find(current_count);
      this->callbacks.erase(it);
    };
    this->count++;
    return deregisterer_func;
  }
};

class StreamMaster
{
private:
  StreamMaster();
  ~StreamMaster();
  StreamMaster(const StreamMaster&) = delete;
  StreamMaster& operator=(const StreamMaster&) = delete;

public:
  static StreamMaster& getInstance();
  static bool ok();

  static bool init();

private:
  static std::atomic<bool> is_ok;
  static void sigintHandler(int);
  template <typename dataT>
  std::shared_ptr<RegisterConnector<dataT>>
  findOrCreateConnector(const std::string& topic_name)
  {
    std::lock_guard<std::mutex> lck(map_mtx);
    auto component_itr = topic_to_connectors.find(topic_name);
    std::shared_ptr<RegisterConnector<dataT>> reg_connector_ptr;
    if (component_itr == topic_to_connectors.end())
    {
      reg_connector_ptr = std::make_shared<RegisterConnector<dataT>>();
      topic_to_connectors[topic_name] = reg_connector_ptr;
      topic_to_registered_count[topic_name] = 1;
    }
    else
    {
      std::shared_ptr<Connector> component_ptr = component_itr->second;
      reg_connector_ptr =
          std::dynamic_pointer_cast<RegisterConnector<dataT>>(component_ptr);
      if (!reg_connector_ptr)
      {
        throw std::logic_error{
            "Connector is not of type RegisterConnector<dataT>: " + topic_name};
      }
      topic_to_registered_count.at(topic_name)++;
    }
    return reg_connector_ptr;
  }
  std::mutex map_mtx;
  std::map<std::string, std::shared_ptr<Connector>> topic_to_connectors;
  std::map<std::string, int> topic_to_registered_count;

  void deregisterFunction(const std::string& topic_name)
  {
    std::lock_guard<std::mutex> lck(map_mtx);
    int decremented_count = topic_to_registered_count.at(topic_name) - 1;
    topic_to_registered_count[topic_name] = decremented_count;
    if (decremented_count == 0)
    {
      topic_to_registered_count.erase(
          topic_to_registered_count.find(topic_name));
      topic_to_connectors.erase(topic_to_connectors.find(topic_name));
    }
  }

public:
  template <typename dataT>
  std::shared_ptr<PublishConnector<dataT>> registerPublisher(
      const std::string& topic_name, std::function<void()>& deregister_func)
  {
    auto reg_connector_ptr = findOrCreateConnector<dataT>(topic_name);

    deregister_func = [this, topic_name]() { deregisterFunction(topic_name); };
    return reg_connector_ptr; // return as PublishConnector
  }
  template <typename dataT>
  void registerSubscriber(
      const std::string& topic_name,
      const std::function<void(const dataT&)>& callback,
      std::function<void()>& deregister_func)
  {
    auto reg_connector_ptr = findOrCreateConnector<dataT>(topic_name);
    auto connector_deregister_func = reg_connector_ptr->registerCb(callback);

    deregister_func = [this, topic_name, connector_deregister_func]() {
      connector_deregister_func();
      deregisterFunction(topic_name);
    };
    return;
  }
};
} // namespace stream
} // namespace medrct
