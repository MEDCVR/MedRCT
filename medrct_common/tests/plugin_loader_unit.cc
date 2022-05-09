/**
 * @file plugin_loader_unit.h
 * @brief Plugin Loader Unit Tests
 *
 * @author Levi Armstrong
 * @date March 25, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief
 * @author Radian Gondokaryono
 * @par Modified to use medrct class_loader.hh header file which changes
 * namespace tesseract_common to medrct_loader. Also remove plugin_lib related
 * tests. Removed SECTION_NAME definition.
 */

#include <gtest/gtest.h>

#include <medrct_common/loader/class_loader.hh>
#include <medrct_common/log.hh>
#include "dummy_plugins/test_plugin_base.h"

TEST(TesseractClassLoaderUnit, LoadTestPlugin) // NOLINT
{
  using medrct_loader::ClassLoader;
  using medrct_loader::TestPluginBase;

  const std::string lib_name = "medrct_common_test_plugin_multiply";
  const std::string lib_dir = std::string(TEST_PLUGIN_DIR);
  const std::string symbol_name = "plugin";
  {
    std::vector<std::string> sections =
        ClassLoader::getAvailableSections(lib_name, lib_dir);
    EXPECT_EQ(sections.size(), 1);
    EXPECT_EQ(sections.at(0), "TestBase");

    sections = ClassLoader::getAvailableSections(lib_name, lib_dir, true);
    EXPECT_TRUE(sections.size() > 1);
  }

  {
    std::vector<std::string> symbols =
        ClassLoader::getAvailableSymbols("TestBase", lib_name, lib_dir);
    EXPECT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), symbol_name);
  }

  {
    EXPECT_TRUE(ClassLoader::isClassAvailable(symbol_name, lib_name, lib_dir));
    auto plugin = ClassLoader::createSharedInstance<TestPluginBase>(
        symbol_name, lib_name, lib_dir);
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }

// For some reason on Ubuntu 18.04 it does not search the current directory when
// only the library name is provided
#if BOOST_VERSION > 106800
  {
    EXPECT_TRUE(ClassLoader::isClassAvailable(symbol_name, lib_name));
    auto plugin = ClassLoader::createSharedInstance<TestPluginBase>(
        symbol_name, lib_name);
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }
#endif

  {
    EXPECT_FALSE(
        ClassLoader::isClassAvailable(symbol_name, lib_name, "does_not_exist"));
    EXPECT_FALSE(
        ClassLoader::isClassAvailable(symbol_name, "does_not_exist", lib_dir));
    EXPECT_FALSE(
        ClassLoader::isClassAvailable("does_not_exist", lib_name, lib_dir));
  }

  {
    EXPECT_FALSE(ClassLoader::isClassAvailable(symbol_name, "does_not_exist"));
    EXPECT_FALSE(ClassLoader::isClassAvailable("does_not_exist", lib_name));
  }

  {
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(
        symbol_name, lib_name, "does_not_exist"));
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(
        symbol_name, "does_not_exist", lib_dir));
    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(
        "does_not_exist", lib_name, lib_dir));
  }

  {
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(
        symbol_name, "does_not_exist")); // NOLINT
    EXPECT_ANY_THROW(ClassLoader::createSharedInstance<TestPluginBase>(
        "does_not_exist", lib_name)); // NOLINT
  }
}

TEST(TesseractClassLoaderUnit, LoadTestPluginYaml) // NOLINT
{
  std::string yaml_params_string =
      R"(test_load:
          lib_name: medrct_common_test_plugin_multiply
          package_name: medrct_common
          class_name: plugin)";
  YAML::Node config = YAML::Load(yaml_params_string);
  auto plugin =
      medrct_loader::createSharedInstance<medrct_loader::TestPluginBase>(
          config["test_load"]);
  EXPECT_TRUE(plugin != nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
