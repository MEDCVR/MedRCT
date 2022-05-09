#include <gtest/gtest.h>

#include <memory>

#include <medrct_common/log.hh>
#include <medrct_common/types.hh>
#include <medrct_common/loader/class_loader.hh>

#include <medrct_environment/description/kinematics_tree.hh>
#include <medrct_environment/kinematics/forward_kinematics.hh>
#include <medrct_environment/kinematics/kinematics_factory.hh>

using namespace medrct;
using namespace medrct::env;

env::ForwardKinematics::Ptr create_fwd(const YAML::Node& config)
{
  auto forward_kinematics_factory =
      medrct_loader::createSharedInstance<env::ForwardKinematicsFactory>(
          config["test_load"]);
  // ASSERT_TRUE(forward_kinematics_factory != nullptr);
  KinematicsTree kin_tree;
  auto forward_kin =
      forward_kinematics_factory->create(kin_tree, config["test_load"]);
  return forward_kin;
}

TEST(PsmFactoryTest, testFKIKOutput)
{
  std::string yaml_params_string =
      R"(test_load:
          lib_name: medrct_dvrk
          package_name: medrct_dvrk
          class_name: PsmForwardKinematicsFactory
          tool_name: LND400006)";
  YAML::Node config = YAML::Load(yaml_params_string);

  auto forward_kin = create_fwd(config);
  ASSERT_TRUE(forward_kin != nullptr);

  Transform tf;
  forward_kin->computeFK(tf, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  medrctlog::info("Transform {}", tf);
}
