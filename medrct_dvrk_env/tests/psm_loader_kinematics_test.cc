#include <gtest/gtest.h>

#include <memory>

#include <medrct/log.hh>
#include <medrct/types/types.hh>
#include <medrct/loader/class_loader.hh>

#include <medrct_env/description/kinematics_tree.hh>
#include <medrct_env/kinematics/forward_kinematics.hh>
#include <medrct_env/kinematics/kinematics_factory.hh>

using namespace medrct;
using namespace medrct::env;

env::ForwardKinematics::Ptr create_fwd(const YAML::Node& config)
{
  auto forward_kinematics_factory =
      medrct_loader::createSharedInstance<env::ForwardKinematicsFactory>(
          config["test_load"]);
  // ASSERT_TRUE(forward_kinematics_factory != nullptr);
  KinematicsTree::Ptr kin_tree;
  auto forward_kin =
      forward_kinematics_factory->create(kin_tree, config["test_load"]);
  return forward_kin;
}

TEST(PsmFactoryTest, testFKIKOutput)
{
  std::string yaml_params_string =
      R"(test_load:
          lib_name: medrct_dvrk_env
          package_name: medrct_dvrk_env
          class_name: PsmForwardKinematicsFactory
          config:
            tool_name: LND400006)";
  YAML::Node config = YAML::Load(yaml_params_string);

  auto forward_kin = create_fwd(config);
  ASSERT_TRUE(forward_kin != nullptr);

  Transform tf;
  forward_kin->computeFK(tf, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  medrctlog::info("Transform {}", tf);
}
