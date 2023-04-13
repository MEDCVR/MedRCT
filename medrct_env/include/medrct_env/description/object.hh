#pragma once

#include <vector>
#include <string>

#include <medrct/types/types.hh>

namespace medrct
{
namespace env
{
enum class object_t
{
  LINE,
  PLANE,
  BOX,
  MESH,
  SPHERE,
  CYLINDER,
  NONE = -1,
};

// inline std::string EnumToString(const object_t& cot)
// {
//   if (cot == object_t::BOX)
//     return "box";
//   if (cot == object_t::MESH)
//     return "mesh";
//   if (cot == object_t::SPHERE)
//     return "sphere";
//   if (cot == object_t::CYLINDER)
//     return "cylinder";
//   return "none";
// }

// inline object_t ObjectStringToEnum(const std::string& s)
// {
//   if (s == "box")
//     return object_t::BOX;
//   if (s == "mesh")
//     return object_t::MESH;
//   if (s == "sphere")
//     return object_t::SPHERE;
//   if (s == "cylinder")
//     return object_t::CYLINDER;
//   return object_t::NONE;
// }

class Object
{
public:
  Object() = default;
  ~Object() = default;
  std::string name;
  std::string frame_id;
  Transform origin;
  object_t type = object_t::NONE;
  std::string mesh_path;
  // Box, Plane only uses x&y, Line only uses x
  real_t& x();
  real_t& y();
  real_t& z();
  // Sphere, Cylinder
  real_t& radius();
  // Cylinder
  real_t& length();
  void rescale(const real_t factor);

private:
  Vector3 dimension;
};
} // namespace env
} // namespace medrct
