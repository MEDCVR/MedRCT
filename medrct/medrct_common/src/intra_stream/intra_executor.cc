
#include <medrct/intra_stream/intra_executor.hh>
#include <medrct/log.hh>

namespace medrct
{
namespace stream
{

ExecutionProcess::ExecutionProcess()
{
}
ExecutionProcess::~ExecutionProcess()
{
}
void ExecutionProcess::set(
    const std::shared_ptr<ThreadNotifier> thread_notifier)
{
  this->thread_notifier = thread_notifier;
}

Thread::Thread()
{
  thread_notifier = std::make_shared<ThreadNotifier>();
}

Thread::~Thread()
{
}

void Thread::registerProcess(
    const std::shared_ptr<ExecutionProcess> exec_process)
{
  if (!thread)
  {
    thread = std::make_unique<std::thread>(&Thread::executeProcesses, this);
  }
  exec_process->set(thread_notifier);
  std::lock_guard<std::mutex> vec_lock(vectors_mtx);
  exec_processes.push_back(exec_process);
  process_bool_returns.push_back(false);
}
void Thread::stop()
{
  {
    std::unique_lock<std::mutex> thread_lock(thread_notifier->mtx);
    is_stop.store(true);
    thread_notifier->cv.notify_all();
  }
  if (thread)
  {
    thread->join();
  }
}

inline bool IsAllFalse(const std::vector<bool>& vec)
{
  for (const auto& ele : vec)
    if (ele)
      return false;
  return true;
}

void Thread::executeProcesses()
{
  // If this thread does not stop,
  // The process does not stop.
  while (!is_stop.load())
  {
    bool is_all_false;
    {
      std::lock_guard<std::mutex> vec_lock(vectors_mtx);
      for (unsigned int i = 0; i < exec_processes.size(); ++i)
      {
        process_bool_returns[i] = exec_processes[i]->processOneData();
      }
      is_all_false = IsAllFalse(process_bool_returns); //
    }
    if (is_all_false)
    {
      // Lock the incoming data only when counting the buffer size
      std::unique_lock<std::mutex> thread_lock(thread_notifier->mtx);
      int buffer_size = 0;
      {
        std::lock_guard<std::mutex> vec_lock(vectors_mtx);
        for (const auto& exec_process : exec_processes)
        {
          buffer_size = exec_process->getBufferSize();
        }
      }
      // if is_stop true, skip and exit while loop
      if (buffer_size == 0 && !is_stop.load())
      {
        thread_notifier->cv.wait(thread_lock);
      }
    }
  }
}

PipelineExecutor::PipelineExecutor(unsigned int max_num_threads)
    : max_num_threads(max_num_threads)
{
  exec_process_count = 0;
}

PipelineExecutor::~PipelineExecutor()
{
  for (auto& thread : threads)
  {
    thread->stop();
  }
}

void PipelineExecutor::registerProcess(
    const std::shared_ptr<ExecutionProcess> exec_process)
{
  if (threads.size() <= max_num_threads)
  {
    threads.push_back(std::move(std::make_unique<Thread>()));
    threads.back()->registerProcess(exec_process);
  }
  else
  {
    threads[exec_process_count % max_num_threads]->registerProcess(
        exec_process);
  }
  exec_process_count++;
}
} // namespace stream
} // namespace medrct
