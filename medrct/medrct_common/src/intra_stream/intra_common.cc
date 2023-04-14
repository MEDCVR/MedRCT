#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <medrct/intra_stream/intra_common.hh>
#include <medrct/stream/condition_wait.hh>

namespace medrct
{
namespace stream
{

Connector::~Connector()
{
}

StreamMaster::StreamMaster()
{
}
StreamMaster::~StreamMaster()
{
  is_ok.store(false);
}
StreamMaster& StreamMaster::getInstance()
{
  static StreamMaster s;
  return s;
}
bool StreamMaster::ok()
{
  return is_ok.load();
}
std::atomic<bool> StreamMaster::is_ok{true};
bool StreamMaster::init()
{
  // A ctrl+c handler, to stop all `while(StreamMaster::ok()) ...`
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = &StreamMaster::sigintHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  ConditionWait::getInstance().setIsRunningFunction(
      [&is_ok]() { return is_ok.load(); });
  return true;
}
void StreamMaster::sigintHandler(int)
{
  is_ok.store(false);
}
} // namespace stream
} // namespace medrct
