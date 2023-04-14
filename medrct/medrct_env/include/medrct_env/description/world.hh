#pragma once

#include <memory>
#include <unordered_map>
#include "kinematics_tree.hh"

namespace medrct
{
namespace env
{
// A Singleton
class World
{
private:
  World();
  ~World();
  World(const World&) = delete;
  World& operator=(const World&) = delete;

public:
  static World& getInstance();
  void parseURDFFile(const std::string& urdf_file_path);
  KinematicsTree::ConstPtr getKinematicsTree(const std::string& name);

private:
  std::unordered_map<std::string, KinematicsTree::Ptr> name_to_kinematic_trees;
};
} // namespace env
} // namespace medrct
