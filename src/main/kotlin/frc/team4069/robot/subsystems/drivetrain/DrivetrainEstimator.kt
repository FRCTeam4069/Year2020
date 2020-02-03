package frc.team4069.robot.subsystems.drivetrain

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpiutil.math.numbers.N2
import edu.wpi.first.wpiutil.math.numbers.N3
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.statespace.ExtendedKalmanFilter
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Twist2d
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.feet

/**
 * An implementation of an extended kalman filter to fuse data from odometry with camera data from SolvePnP
 *
 * [update] should be called frequently, with [measuredPose] being nonnull only when there is a vision target in sight.
 */
class DrivetrainEstimator {
    val trackWidth = 2.3563.feet.value

    /**
     * The extended kalman filter
     */
    val observer = ExtendedKalmanFilter(`3`, `3`, `3`, { x, u -> stateUpdate(x, u, kNominalDt) },
        this::h,
        vec(`3`).fill(0.3, 1.0, 0.4), // State stddevs
        vec(`3`).fill(2.0, 2.5, 3.0), // Measurement stddevs
        kNominalDt)

    /**
     * State update function.
     * @param x       Robot pose in global coordinate frame, [[x, y, theta]]^T
     * @param inputs  Drivetrain velocities, [[dl, dr, dtheta]]^T
     * @param dt      The time since the last call to update the observer
     */
    private fun stateUpdate(x: Vector<N3>, inputs: Vector<N3>, dt: SIUnit<Second>): Vector<N3> {
        // Matrix form of calculations in DifferentialDriveOdometry
        val mat = mat(`3`, `3`).fill(.5, .5, .0, .0, .0, .0, .0, .0, 1.0)

        val twist = Pose2d().exp((mat * inputs).asTwist()).toVector()
        return twist / dt.value
    }

    /**
     * The output function y = h(x, u)
     */
    private fun h(x: Vector<N3>, u: Vector<N3>): Vector<N3> {
        return x
    }

    /**
     * Updates the state of the extended kalman filter
     *
     * @param dt           The time since the last call
     * @param u            The input velocities as a vector (Identical to those described for [stateUpdate])
     * @param measuredPose The pose of the robot as measured using data from the Limelight. This value should only be nonnull if a vision target is in sight.
     */
    fun update(dt: SIUnit<Second>, u: Vector<N3>, measuredPose: Pose2d? = null) {
        observer.predict(u, dt) { x, u -> stateUpdate(x, u, dt) }
        // Implication is that a vision target is in sight
        if(measuredPose != null) {
            observer.correct(u, measuredPose.toVector())
        }
    }

    // Utility functions to convert between vectors and wpilib geometry classes
    private fun Vector<N3>.asTwist(): Twist2d {
        return Twist2d(this[0].meter, this[1].meter, this[2].radian)
    }

    private fun Pose2d.toVector(): Vector<N3> {
        return vec(`3`).fill(translation.x, translation.y, rotation.radians)
    }

    companion object {
        // The nominal loop time of the filter
        val kNominalDt = 0.01.second
    }
}