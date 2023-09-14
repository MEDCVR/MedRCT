#pragma once

#include <iostream>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "precision.hh"

namespace medrct
{
using Vector3 = Eigen::Matrix<real_t, 3, 1>;
using AngleAxis = Eigen::AngleAxis<real_t>;
using Quaternion = Eigen::Quaternion<real_t>;
using Rotation = Eigen::Matrix<real_t, 3, 3>;
using Transform = Eigen::Transform<real_t, 3, Eigen::Isometry>;

template <class T>
using Ref = Eigen::Ref<T>;

static inline Quaternion FromRPY(const Vector3& rpy)
{
  Quaternion eig_quat = Eigen::AngleAxis<real_t>(rpy.y(), Vector3::UnitY()) *
                        Eigen::AngleAxis<real_t>(rpy.x(), Vector3::UnitX());
  eig_quat = Eigen::AngleAxis<real_t>(rpy.z(), Vector3::UnitZ()) * eig_quat;
  return Quaternion(eig_quat);
}

struct Joy
{
  std::vector<int> buttons;
};

static inline Quaternion FromMatrix(
    real_t m00,
    real_t m01,
    real_t m02,
    real_t m10,
    real_t m11,
    real_t m12,
    real_t m20,
    real_t m21,
    real_t m22)
{
  Eigen::Matrix<real_t, 3, 3> mat;
  mat << m00, m01, m02, m10, m11, m12, m20, m21, m22;
  return Quaternion(mat);
}

static inline std::ostream&
operator<<(std::ostream& stream, const Transform& isometry)
{
  stream << "Translation: " << std::endl << isometry.translation() << std::endl;
  stream << "Rotation: " << std::endl << isometry.rotation();

  return stream;
}

} // namespace medrct
