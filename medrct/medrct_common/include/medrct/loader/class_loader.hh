#pragma once

#include <yaml-cpp/yaml.h>
#include <medrct/file_paths.hh>
#include "tesseract/class_loader.h"

namespace medrct_loader = tesseract_common;

// clang-format off
#define MEDRCT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, SECTION)      \
    TESSERACT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, SECTION)

#define MEDRCT_ADD_PLUGIN(DERIVED_CLASS, ALIAS)                         \
    TESSERACT_ADD_PLUGIN(DERIVED_CLASS, ALIAS)
// clang-format on

namespace tesseract_common
{

template <typename T>
std::shared_ptr<T> createSharedInstance(const YAML::Node& loader_config)
{
  std::string lib_name;
  std::string package_name;
  std::string class_name; // symbol name
  if (YAML::Node m = loader_config["lib_name"])
    lib_name = m.as<std::string>();
  else
    throw std::runtime_error("No [lib_name] key in config");
  if (YAML::Node m = loader_config["package_name"])
    package_name = m.as<std::string>();
  else // Try package name as lib name
    package_name = lib_name;
  if (YAML::Node m = loader_config["class_name"])
    class_name = m.as<std::string>();
  else
    throw std::runtime_error("No [class_name] key in config");

  std::string lib_dir;
  lib_dir = medrct::GetBuildDirectoryPath() + "/" + package_name;
  // Should throw if problem
  return medrct_loader::ClassLoader::createSharedInstance<T>(
      class_name, lib_name, lib_dir);
}
} // namespace tesseract_common
