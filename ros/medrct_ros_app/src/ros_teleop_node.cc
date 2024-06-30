#include <memory>
#include <yaml-cpp/yaml.h>

#include <medrct/file_paths.hh>
#include <medrct/log.hh>
#include <medrct/controller/controller_manager_config.hh>
#include <medrct/controller/controller_manager.hh>
#include <medrct_ros/ros_singleton.hh>
#include <medrct_ros/ros_stream_factory.hh>
#include <ros/ros.h>

using namespace medrct::stream;
using namespace medrct::controller;

int main(int argc, char** argv)
{
  if (!RosSingleton::getInstance().init(argc, argv, "medrct_ros_teleop_node"))
  {
    return -1;
  }

  std::string relative_path_to_yaml;
  RosSingleton::getInstance().getNodeHandle()->getParam(
    "/medrct_ros_teleop_node/config", relative_path_to_yaml);

  std::string full_path_to_yaml =
  medrct::GetInstallDirectoryPath() + "/" + relative_path_to_yaml;

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

  RosStreamFactory::Ptr ros_stream_factory =
      std::make_shared<RosStreamFactory>();

  ControllerManagerConfig cmc =
      FromYAMLControllerManagerConfig(config, *ros_stream_factory);
  ControllerManager cm;
  if (!cm.init(cmc))
  {
    medrctlog::error("Controller manager failed to initialize.");
    return -1;
  }
  std::thread thd([]() {
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
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
  medrctlog::info("Controller stopped");
  return 0;
}
