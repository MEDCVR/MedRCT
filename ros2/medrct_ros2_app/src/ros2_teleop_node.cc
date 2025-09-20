#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct/file_paths.hh>
#include <medrct/log.hh>
#include <medrct/controller/controller_manager_config.hh>
#include <medrct/controller/controller_manager.hh>
#include <medrct_ros2/ros2_singleton.hh>
#include <medrct_ros2/ros2_stream_factory.hh>
#include <rclcpp/rclcpp.hpp>

using namespace medrct::stream;
using namespace medrct::controller;

int main(int argc, char** argv)
{
  if (!Ros2Singleton::getInstance().init(argc, argv, "medrct_ros2_teleop_node"))
  {
    return -1;
  }

  std::string relative_path_to_yaml;
  auto node = Ros2Singleton::getInstance().getNodeHandle();
  node->declare_parameter<std::string>("config", "NOT_SPECIFIED_CONFIG");
  node->get_parameter(
      "config", relative_path_to_yaml);

  std::string full_path_to_yaml = medrct::GetInstallDirectoryPath() +
                                  "/medrct_ros2_app/share/medrct_ros2_app/" +
                                  relative_path_to_yaml;

  medrctlog::info("Config Path: {}", full_path_to_yaml);

  YAML::Node config;
  try
  {
    config = YAML::LoadFile(full_path_to_yaml);
  }
  catch (const std::exception& e)
  {
    medrctlog::error("Bad Yaml Path: {}", full_path_to_yaml);
    throw;
  }

  Ros2StreamFactory::Ptr ros_stream_factory =
      std::make_shared<Ros2StreamFactory>();

  ControllerManagerConfig cmc =
      FromYAMLControllerManagerConfig(config, *ros_stream_factory);
  ControllerManager cm;
  if (!cm.init(cmc))
  {
    medrctlog::error("Controller manager failed to initialize.");
    return -1;
  }
  std::thread thd([&node]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  BasicControllerManagerCommunicator communicator;
  YAML::Node communicator_config = config["basic_communicator"];
  if (!communicator_config)
  {
    medrctlog::error("No [basic_communicator] key in config");
    return -1;
  }
  BasicControllerManagerCommunicatorConfig bcmc_cfg =
      FromYAMLBasicControllerCommunicatorConfig(
          communicator_config, *ros_stream_factory);
  if (!communicator.init(std::move(cm), bcmc_cfg))
  {
    return -1;
  }
  medrctlog::info("Controller is running");
  thd.join();
  rclcpp::shutdown();
  medrctlog::info("Controller stopped");
  return 0;
}
