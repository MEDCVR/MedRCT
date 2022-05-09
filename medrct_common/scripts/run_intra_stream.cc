#include <iostream>
#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#include <medrct_common/intra/intra.hh>

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
