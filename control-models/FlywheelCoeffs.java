import edu.wpi.first.wpilibj.controller.StateSpaceControllerCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpaceLoop;
import edu.wpi.first.wpilibj.controller.StateSpaceObserverCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpacePlantCoeffs;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;

public class FlywheelCoeffs {
  public static StateSpacePlantCoeffs<N2, N1, N1> makeFlywheelPlantCoeffs() {
    Matrix<N2, N2> A = MatrixUtils.mat(Nat.N2(), Nat.N2()).fill(0.9675557895260317, 1.3303056835711813, 0.0, 0.0);
    Matrix<N2, N1> B = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(1.3303056835711813, 0.0);
    Matrix<N1, N2> C = MatrixUtils.mat(Nat.N1(), Nat.N2()).fill(1.0, 0.0);
    Matrix<N1, N1> D = MatrixUtils.mat(Nat.N1(), Nat.N1()).fill(0.0);
    return new StateSpacePlantCoeffs<N2, N1, N1>(Nat.N2(), Nat.N1(), Nat.N1(), A, B, C, D);
  }

  public static StateSpaceControllerCoeffs<N2, N1, N1>
    makeFlywheelControllerCoeffs() {
    Matrix<N1, N2> K = MatrixUtils.mat(Nat.N1(), Nat.N2()).fill(0.18409390097067566, 1.0);
    Matrix<N1, N2> Kff = MatrixUtils.mat(Nat.N1(), Nat.N2()).fill(0.7517069289785474, 0.0);
    Matrix<N1, N1> Umin = MatrixUtils.mat(Nat.N1(), Nat.N1()).fill(-12.0);
    Matrix<N1, N1> Umax = MatrixUtils.mat(Nat.N1(), Nat.N1()).fill(12.0);
    return new StateSpaceControllerCoeffs<N2, N1, N1>(K, Kff, Umin, Umax);
  }

  public static StateSpaceObserverCoeffs<N2, N1, N1>
    makeFlywheelObserverCoeffs() {
    Matrix<N2, N1> K = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(0.795541093137215, 1.6517591872016555e-16);
    return new StateSpaceObserverCoeffs<N2, N1, N1>(K);
  }

  public static StateSpaceLoop<N2, N1, N1> makeFlywheelLoop() {
    return new StateSpaceLoop<N2, N1, N1>(makeFlywheelPlantCoeffs(),
                                          makeFlywheelControllerCoeffs(),
                                          makeFlywheelObserverCoeffs());
  }
}
