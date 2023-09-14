#pragma once

#include <string>

#include <medrct_env/description/object.hh>

namespace medrct
{
namespace env
{
class Vizualizer
{
public:
  Vizualizer() = default;
  virtual ~Vizualizer() = default;
  void setObjectGroup(const ObjectGroup& object_group);
  void updateTransforms(
      const std::string& group_name,
      const std::unordered_map<std::string, Transform> object_transforms);
  virtual void publish() = 0;

private:
  std::unordered_map<std::string, ObjectGroup> object_groups;
};
} // namespace env
} // namespace medrct
