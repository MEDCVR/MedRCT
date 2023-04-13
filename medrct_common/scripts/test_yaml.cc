#include <yaml-cpp/yaml.h>
#include <medrct/log.hh>

int main(int, char**)
{
  std::string yaml_params_string =
      R"(kinematic_plugins:
           a_number: 2012
           inv_kin_plugins:
             manipulator:
               default: URInvKin
               plugins:
                 URInvKin:
                   class: URInvKinFactory
                   config:
                     base_link: base_link
                     tip_link: tool0
                     params:
                       d1: 0.1273
                       a2: -0.612
                       a3: -0.5723
                       d4: 0.163941
                       d5: 0.1157
                       d6: 0.0922)";
  YAML::Node config = YAML::Load(yaml_params_string);
  medrctlog::info(config["kinematic_plugins"]);
  int a_number = config["kinematic_plugins"]["a_number"].as<int>();
  int b_number = a_number + 2;
  medrctlog::info("{}", b_number);
}
