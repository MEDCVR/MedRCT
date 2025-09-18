#include <eigen3/Eigen/Geometry>
#include <medrct/log.hh>

using real_t = float;
using Vector3 = Eigen::Matrix<real_t, 3, 1>;
using Quaternion = Eigen::Quaternion<real_t>;
using Rotation = Eigen::Matrix<real_t, 3, 3>;
using TransformIsometry3 = Eigen::Transform<real_t, 3, Eigen::Isometry>;
// using TransformAffine3 = Eigen::Transform<real_t, 3, Eigen::Affine>;

template <class T>
using Ref = Eigen::Ref<T>;

void Print(const TransformIsometry3& mat)
{
  medrctlog::info("position: \n{}", TypeToString(mat.translation()));
  medrctlog::info("rotation: \n{}", TypeToString(mat.linear()));
}

void SetPosition(TransformIsometry3& mat, const Vector3& pos)
{
  mat.translation() = pos;
  mat.translation() = Vector3(0.7, 0.4, 1.1);
}

void SetRotation(TransformIsometry3& mat, const Quaternion& rot)
{
  mat.linear() = rot.toRotationMatrix();
}

void MalModifyRotation(Ref<Rotation> rot)
{
  rot(1, 0) = 1.294;
}

int main(int, char**)
{
  Vector3 pos;
  pos << 0.0, -2.0, 1.0;
  Vector3 pos2 = Vector3(1, -3, 0.5);
  pos = pos + pos2;

  Quaternion quat;
  quat.setFromTwoVectors(Vector3(0, 1, 0), pos);

  TransformIsometry3 aff = TransformIsometry3::Identity();
  SetPosition(aff, pos);
  SetRotation(aff, quat);

  Print(aff); // Print object
  Print(TransformIsometry3::Identity());

  Vector3 pos3 = aff * pos2;
  medrctlog::info("{}", TypeToString(pos3));

  Rotation r = quat.toRotationMatrix();
  (void)r;

  medrctlog::info("original rotation: \n{}", TypeToString(r));
  MalModifyRotation(r);
  medrctlog::info("modified rotation: \n{}", TypeToString(r));

  TransformIsometry3 t = TransformIsometry3::Identity();
  t.fromPositionOrientationScale(pos, quat, Vector3(1, 1, 1));
  Print(t);
}
