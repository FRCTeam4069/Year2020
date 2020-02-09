package frc.team4069.robot.subsystems.drivetrain

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Twist2d
import edu.wpi.first.wpiutil.CircularBuffer
import edu.wpi.first.wpiutil.math.numbers.N3
import frc.team4069.saturn.lib.localization.TimeInterpolatableBuffer
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.statespace.ExtendedKalmanFilter
import frc.team4069.saturn.lib.mathematics.statespace.UnscentedKalmanFilter
import frc.team4069.saturn.lib.mathematics.twodim.geometry.InterpolatablePose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Twist2d
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.derived.Volt

/**
 * An implementation of an extended kalman filter to fuse data from odometry with camera data from SolvePnP
 *
 * [update] should be called frequently, with [measuredPose] being nonnull only when there is a vision target in sight.
 */
class DrivetrainEstimator {
    val trackWidth = 2.3563.feet.value

    private val pastPoseBuffer = TimeInterpolatableBuffer<InterpolatablePose2d>()
    private val pastInputs = mutableMapOf<Pose2d, Triplet<SIUnit<Second>, Vector<N3>, SIUnit<Second>>>()

    /**
     * The extended kalman filter
     */
    val observer = ExtendedKalmanFilter(`3`, `3`, `3`, this::stateUpdate,
        this::h,
        vec(`3`).fill(0.01, 0.2, 0.001), // State stddevs
        vec(`3`).fill(0.05, 0.1, 0.01),
        kNominalDt,
        useRungeKutta = false
        )

    /**
     * State update function.
     * @param x       Robot pose in global coordinate frame, [[x, y, theta]]^T
     * @param inputs  Drivetrain velocities, [[dl, dr, dtheta]]^T
     */
    private fun stateUpdate(x: Vector<N3>, inputs: Vector<N3>): Vector<N3> {
        // Matrix form of calculations in DifferentialDriveOdometry
        val mat = mat(`3`, `3`).fill(.5, .5, .0, .0, .0, .0, .0, .0, 1.0)

        val twist = Pose2d(x[0].meter, x[1].meter, x[2].radian).exp((mat * inputs).asTwist())
        // theta_2 - theta_1 + theta_1 = theta_2; new angle of robot
        return Pose2d(twist.translation, Rotation2d(inputs[2] + x[2])).toVector()
    }

    /**
     * The output function y = h(x, u)
     */
    private fun h(x: Vector<N3>, u: Vector<N3>): Vector<N3> {
        return x
    }

    fun reset(initPose: Pose2d) {
        observer.xHat = vec(`3`).fill(initPose.translation.x, initPose.translation.y, initPose.rotation.radians)
    }

    /**
     * Updates the state of the extended kalman filter
     *
     * @param dt           The time since the last call
     * @param u            The input velocities as a vector (Identical to those described for [stateUpdate])
     * @param measuredPose The pose of the robot as measured using data from the Limelight. This value should only be nonnull if a vision target is in sight.
     */
    fun update(dt: SIUnit<Second>, u: Vector<N3>, measuredPose: Pose2d? = null, measuredPoseTimestamp: SIUnit<Second> = (-1).second) {
        // Implication is that a vision target is in sight
        if(measuredPose != null) {
            // Latency compensation; reset observer pose to the pose when the frame was in sight,
            // add correction factor, and replay all predictions into the future
            val thenPose = pastPoseBuffer[measuredPoseTimestamp]
            observer.xHat = thenPose.pose.toVector()
            observer.correct(u, measuredPose.toVector())
            val firstIdx = pastInputs.keys.indexOf(thenPose.pose)
            val inputsToReplay = pastInputs.values.toList().subList(firstIdx, pastInputs.size - 1)

            for((_, u, dt) in inputsToReplay) {
                observer.predict(u, dt)
            }
        }
        observer.predict(u, dt)

        val now = Timer.getFPGATimestamp().second
        val iter = pastInputs.iterator()
        iter.forEach {
            if(now - it.value.first >= 1.second) {
                iter.remove()
            }
        }
        pastInputs[pose] = Triplet(now, u, dt)
        pastPoseBuffer[now] = InterpolatablePose2d(pose)
    }

    val pose: Pose2d
        get() = Pose2d(observer.xHat[0], observer.xHat[1], Rotation2d(observer.xHat[2]))

    var lastLeft = 0.meter
    var lastRight = 0.meter
    var lastAngle = 0.radian

    fun update(dt: SIUnit<Second>, leftPosition: SIUnit<Meter>, rightPosition: SIUnit<Meter>, rotation: Rotation2d, measuredPose: Pose2d? = null) {
        val dl = (leftPosition - lastLeft).value
        val dr = (rightPosition - lastRight).value
        val dtheta = rotation.radians - lastAngle.value

        update(dt, vec(`3`).fill(dl, dr, dtheta), measuredPose)
        lastLeft = leftPosition
        lastRight = rightPosition
        lastAngle = rotation.radians.radian
    }

    // Utility functions to convert between vectors and wpilib geometry classes
    private fun Vector<N3>.asTwist(): Twist2d {
        return Twist2d(this[0].meter, this[1].meter, this[2].radian)
    }

    private fun Pose2d.toVector(): Vector<N3> {
        return vec(`3`).fill(translation.x, translation.y, rotation.radians)
    }

    private data class Triplet<A, B, C>(val first: A, val second: B, val third: C)

    companion object {
        // The nominal loop time of the filter
        val kNominalDt = 0.01.second
    }
}
