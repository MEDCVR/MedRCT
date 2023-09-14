#include <iostream>
#include <thread>
#include <chrono>

#include <medrct/intra_stream/intra_stream.hh>

using namespace medrct::stream;

int main(int, char**)
{
  StreamMaster::init();
  while (StreamMaster::ok())
  {
    std::cout << "waiting for signal out" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout << "Ctrl-c finished" << std::endl;
}
