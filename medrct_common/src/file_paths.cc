
#include <medrct_common/file_paths.hh>

namespace medrct
{

std::string getBuildDirectoryPath()
{
  static bool is_path_processed = false;
  static std::string bin_dir_path;
  if (!is_path_processed)
  {
    bin_dir_path = PACKAGE_BUILD_DIR;
    std::string delimiter = "/";
    bin_dir_path = bin_dir_path.substr(0, bin_dir_path.rfind(delimiter));
    is_path_processed = true;
  }
  return bin_dir_path;
}

std::string getInstallDirectoryPath()
{
  static bool is_path_processed = false;
  static std::string install_dir_path;
  if (!is_path_processed)
  {
    install_dir_path = PACKAGE_INSTALL_DIR;
    std::string delimiter = "/";
    install_dir_path =
        install_dir_path.substr(0, install_dir_path.rfind(delimiter));
    is_path_processed = true;
  }
  return install_dir_path;
}

} // namespace medrct
