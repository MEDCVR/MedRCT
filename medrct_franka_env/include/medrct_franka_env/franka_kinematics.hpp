// The analytical Franka inverse kinematics used was developed by Yanhao He and
// Steven Liu. Their work can be found here:
// https://github.com/ffall007/franka_analytical_ik

#ifndef FRANKA_KINEMATICS
#define FRANKA_KINEMATICS

#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class FrankaKinematicsSolver
{

public:
  Eigen::Matrix4d
  computeFk(std::array<double, 7> activeJointPos, int linkNumOutput = 8);
  std::array<std::array<double, 4>, 9>
  getDh(std::array<double, 7> joint_values);
  Eigen::Matrix4d
  dhToTFMat(std::array<std::array<double, 4>, 9> dhChain, int i);

  bool computeIk(
      Eigen::Vector3d position,
      Eigen::Quaterniond rotation,
      std::array<double, 7>& q);
  std::array<double, 7> franka_IK_EE_CC(
      std::array<double, 16> O_T_EE_array,
      double q7,
      std::array<double, 7> q_actual_array,
      bool limit,
      bool flange);
  std::array<std::array<double, 7>, 4> franka_IK_EE(
      std::array<double, 16> O_T_EE_array,
      double q7,
      std::array<double, 7> q_actual_array);
};

#endif
