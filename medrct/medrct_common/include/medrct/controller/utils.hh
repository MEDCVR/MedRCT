#pragma once

#include <functional>
#include <unordered_map>
#include <string>

#include <medrct/stream/stream.hh>

namespace medrct
{
namespace controller
{

class DataStore
{
private:
  class Data
  {
  public:
    virtual ~Data() {}
  };
  template <typename T>
  class TData : public Data
  {
  public:
    TData(const T& data) : p_data(data) {}
    virtual ~TData() {}
    void set(const T& data) { p_data = data; }
    T& get() { return p_data; }

  private:
    T p_data;
  };

public:
  DataStore();
  ~DataStore();
  template <typename T>
  void set(const std::string& name, const T& data)
  {
    data_map[name] = std::make_unique<TData<T>>(data);
  }
  template <typename T>
  void update(const std::string& name, const T& data)
  {
    static_cast<TData<T>*>(data_map.at(name).get())->set(data);
  }
  template <typename T>
  T get(const std::string& name) const
  {
    return static_cast<TData<T>*>(data_map.at(name).get())->get();
  }
  bool contains(const std::string& name) const
  {
    return data_map.find(name) != data_map.end();
  }

private:
  std::unordered_map<std::string, std::unique_ptr<Data>> data_map;
};

class StreamMap
{
private:
  struct BufferedFunctions
  {
    std::function<void(DataStore&)> get_data_func;
    std::function<bool()> wait_data_once_func;
    std::function<void()> remove_buffer_func;
  };

public:
  StreamMap();
  virtual ~StreamMap();
  void add(const std::shared_ptr<stream::Stream> stream);
  template <typename T>
  std::shared_ptr<const T> get(const std::string& name) const
  {
    return std::dynamic_pointer_cast<const T>(name_to_streams.at(name));
  }
  template <typename dataT>
  void
  addWithBuffer(const std::shared_ptr<stream::InputStream<dataT>> input_stream)
  {
    std::string name = input_stream->name;
    if (name_to_streams.find(name) != name_to_streams.end())
    {
      throw std::runtime_error("addWithBuffer stream added with same name");
    }
    name_to_streams[name] = input_stream;

    BufferedFunctions bfs;
    bfs.get_data_func = [this, name](DataStore& data_store) {
      stream::InputStream<dataT>* stream_ptr =
          static_cast<stream::InputStream<dataT>*>(
              name_to_streams.at(name).get());
      // data_store.set is slow, do something about it
      if (!data_store.contains(name))
        data_store.set<dataT>(
            stream_ptr->name, stream_ptr->getBuffer().getLatest());
    };
    bfs.wait_data_once_func = [this, name]() {
      stream::InputStream<dataT>* stream_ptr =
          static_cast<stream::InputStream<dataT>*>(
              name_to_streams.at(name).get());
      return stream_ptr->waitForOneData();
    };
    bfs.remove_buffer_func = [this, name]() {
      auto it = name_to_streams.find(name);
      stream::InputStream<dataT>* stream_ptr =
          static_cast<stream::InputStream<dataT>*>(it->second.get());
      stream_ptr->removeBuffer();
      name_to_streams.erase(it);
    };
    buffered_stream_name_to_functions[name] = bfs;
    input_stream->addBuffer();
  }
  bool removeBuffer(const std::string& stream_name);
  bool waitForOneBufferedDataInput(const std::string& name, bool print = false);

  // This will segfault if buffer data is empty because get_data_functions
  // does not check. You must check first if the stream_ptr-> has a buffered
  // data
  void getDataFromAllBufferedStreams(DataStore& data_store);

private:
  std::unordered_map<std::string, std::shared_ptr<stream::Stream>>
      name_to_streams;
  std::unordered_map<std::string, BufferedFunctions>
      buffered_stream_name_to_functions;
};

enum class task_type_t
{
  AGGRAGATE = 0,
  PERIODIC = 1,
  END = 2
};

class Task
{
public:
  Task();
  virtual ~Task();
  virtual void enable() = 0;
  virtual void disable() = 0;
};

template <typename dataT>
class AggragateTask : public Task
{
public:
  AggragateTask(
      StreamMap& input_stream_map,
      std::shared_ptr<stream::InputStream<dataT>> input_callback_stream,
      std::function<void(DataStore&)> process)
      : input_stream_map(input_stream_map),
        input_callback_stream(input_callback_stream),
        process(process)
  {
  }
  virtual ~AggragateTask() {}
  void enable() override
  {
    input_stream_map.removeBuffer(input_callback_stream->name);
    input_callback_stream->addCallback(
        "controller_aggragate",
        std::bind(
            &AggragateTask<dataT>::runFunction, this, std::placeholders::_1));
  }
  void disable() override
  {
    input_callback_stream->removeCallback("controller_aggragate");
    input_stream_map.addWithBuffer(input_callback_stream);
  }

private:
  void runFunction(const dataT& input_data)
  {
    DataStore data_store;
    data_store.set<dataT>(input_callback_stream->name, input_data);
    process(data_store);
  }
  StreamMap& input_stream_map;
  std::shared_ptr<stream::InputStream<dataT>> input_callback_stream;
  std::function<void(DataStore&)> process;
};

} // namespace controller
} // namespace medrct
