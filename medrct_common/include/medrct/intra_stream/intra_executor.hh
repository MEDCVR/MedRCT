#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "intra_common.hh"

namespace medrct
{
namespace stream
{

struct ThreadNotifier
{
  std::mutex mtx;
  std::condition_variable cv;
};

class ExecutionProcess
{
public:
  ExecutionProcess();
  virtual ~ExecutionProcess();
  virtual int getBufferSize() const = 0;
  // Return false to let the thread notify to wait because no more data
  // Return true to say there is still more data to process
  virtual bool processOneData() = 0;
  void set(const std::shared_ptr<ThreadNotifier>
               thread_notifier); // attach to executor
protected:
  std::shared_ptr<ThreadNotifier> thread_notifier;
};

class Thread
{
public:
  Thread();
  ~Thread();
  void registerProcess(const std::shared_ptr<ExecutionProcess> exec_process);
  void stop(); // TODO After you stop, can't run it again
private:
  void executeProcesses();

  std::mutex vectors_mtx;
  std::vector<std::shared_ptr<ExecutionProcess>> exec_processes;
  std::vector<bool> process_bool_returns;

  std::unique_ptr<std::thread> thread;
  std::shared_ptr<ThreadNotifier> thread_notifier;
  std::atomic<bool> is_stop = {false};
};

struct Executor
{
  virtual void
  registerProcess(const std::shared_ptr<ExecutionProcess> exe_process) = 0;
};
// Each ExecutionProcess gets its own Thread, unless the number of execution
// processes are more than the max_num_threads.
class PipelineExecutor : public Executor
{
public:
  PipelineExecutor(
      unsigned int max_num_threads = std::thread::hardware_concurrency());
  ~PipelineExecutor();
  void registerProcess(
      const std::shared_ptr<ExecutionProcess> exec_process) override;

private:
  unsigned int max_num_threads;
  unsigned int exec_process_count;
  std::vector<std::unique_ptr<Thread>> threads;
};
} // namespace stream
} // namespace medrct
