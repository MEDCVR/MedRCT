
#include <medrct_env/description/object.hh>

namespace medrct
{
namespace env
{
real_t& Object::x()
{
  return dimension[0];
}
real_t& Object::y()
{
  return dimension[1];
}
real_t& Object::z()
{
  return dimension[2];
}

// Sphere, Cylinder
real_t& Object::radius()
{
  return dimension[0];
}
// Cylinder
real_t& Object::length()
{
  return dimension[1];
}
void Object::rescale(const real_t factor)
{
  dimension[0] *= factor;
  dimension[1] *= factor;
  dimension[2] *= factor;
}
} // namespace env
} // namespace medrct
