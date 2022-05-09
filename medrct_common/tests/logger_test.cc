#include <gtest/gtest.h>

#include <medrct_common/log.hh>

// Declare a test
TEST(TestLogger, simpleString)
{
  medrctlog::info("Welcome to spdlog!");
  medrctlog::error("Some error message with arg: {}", 1);

  medrctlog::warn("Easy padding in numbers like {:08d}", 12);
  medrctlog::critical(
      "Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
  medrctlog::info("Support for floats {:03.2f}", 1.23456);
  medrctlog::info("Positional args are {1} {0}..", "too", "supported");
  medrctlog::info("{:<30}", "left aligned");
}
