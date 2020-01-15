#include "subsystems/FlywheelCoeffs.hpp"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<1, 1, 1> MakeFlywheelPlantCoeffs() {
  Eigen::Matrix<double, 1, 1> A;
  A(0, 0) = 0.9975101382517066;
  Eigen::Matrix<double, 1, 1> B;
  B(0, 0) = 0.13636840996605396;
  Eigen::Matrix<double, 1, 1> C;
  C(0, 0) = 1.0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0.0;
  return frc::StateSpacePlantCoeffs<1, 1, 1>(A, B, C, D);
}

frc::StateSpaceControllerCoeffs<1, 1, 1> MakeFlywheelControllerCoeffs() {
  Eigen::Matrix<double, 1, 1> K;
  K(0, 0) = 0.21880790624410937;
  Eigen::Matrix<double, 1, 1> Kff;
  Kff(0, 0) = 0.00784641573077454;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<1, 1, 1>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<1, 1, 1> MakeFlywheelObserverCoeffs() {
  Eigen::Matrix<double, 1, 1> K;
  K(0, 0) = 0.999900019945291;
  return frc::StateSpaceObserverCoeffs<1, 1, 1>(K);
}

frc::StateSpaceLoop<1, 1, 1> MakeFlywheelLoop() {
  return frc::StateSpaceLoop<1, 1, 1>(MakeFlywheelPlantCoeffs(),
                                      MakeFlywheelControllerCoeffs(),
                                      MakeFlywheelObserverCoeffs());
}
