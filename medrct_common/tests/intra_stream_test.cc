#include <gtest/gtest.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <medrct_common/interface/stream.hh>
#include <medrct_common/intra/intra.hh>

using namespace medrct::stream;

struct IntCallbackChecker
{
  void inline publish(std::shared_ptr<OutputStream<int>> output_js)
  {
    for (const int int_data : int_datas)
    {
      output_js->publish(int_data);
    }
  }
  void storeCallback(const int& data)
  {
    returned_int_datas.push_back(data);
    counter++;
    if (counter >= int_datas.size() - 1)
    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.notify_all();
    }
  }

  void checkResults()
  {
    ASSERT_EQ(returned_int_datas.size(), int_datas.size());
    for (unsigned int i = 0; i < int_datas.size(); ++i)
    {
      EXPECT_EQ(returned_int_datas[i], int_datas[i]);
    }
  }

  const std::vector<int> int_datas = {1, 2, 4, 7};
  std::vector<int> returned_int_datas;
  unsigned int counter = 0;

  std::mutex mtx;
  std::condition_variable cv;
};

TEST(TestSharedStream, testPublishSubscribe)
{
  StreamMaster::init();

  std::shared_ptr<OutputStream<int>> output_int =
      std::make_shared<SharedOutputStream<int>>("integer", "output_int");
  std::shared_ptr<SharedInputStream<int>> input_int =
      std::make_shared<SharedInputStream<int>>("integer", "input_int");

  IntCallbackChecker icc;
  input_int->addCallback(
      "only_callback",
      std::bind(
          &IntCallbackChecker::storeCallback, &icc, std::placeholders::_1));

  PipelineExecutor pe;
  pe.registerProcess(input_int);

  std::thread thd([&icc]() {
    std::unique_lock<std::mutex> lk(icc.mtx);
    icc.cv.wait_until(
        lk,
        std::chrono::system_clock::now() +
            std::chrono::seconds(10)); // 10s timeouts
  });

  icc.publish(output_int);
  thd.join();
  icc.checkResults();
}
