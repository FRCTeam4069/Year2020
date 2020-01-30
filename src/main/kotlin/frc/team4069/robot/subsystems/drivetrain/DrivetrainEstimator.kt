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

class DrivetrainEstimator {
    val trackWidth = 2.3563.feet.value

    val observer = ExtendedKalmanFilter(`3`, `2`, `3`, { x, u -> stateUpdate(x, u, kNominalDt) },
        this::h,
        vec(`3`).fill(0.3, 1.0, 0.4),
        vec(`3`).fill(2.0, 2.5, 3.0), kNominalDt)

    private fun stateUpdate(x: Vector<N3>, inputs: Vector<N2>, dt: SIUnit<Second>): Vector<N3> {
        val mat = mat(`3`, `2`).fill(.5, .5, .0, .0, -1/(2*trackWidth), 1/(2*trackWidth)) * dt.value

        val twist = Pose2d().exp((mat * inputs).asTwist()).toVector()
        return twist / dt.value
    }

    private fun h(x: Vector<N3>, u: Vector<N2>): Vector<N3> {
        return x
    }

    fun update(dt: SIUnit<Second>, u: Vector<N2>, measuredPose: Pose2d? = null) {
        observer.predict(u, dt) { x, u -> stateUpdate(x, u, dt) }
        // Implication is that a vision target is in sight
        if(measuredPose != null) {
            observer.correct(u, measuredPose.toVector())
        }
    }

    private fun Vector<N3>.asTwist(): Twist2d {
        return Twist2d(this[0].meter, this[1].meter, this[2].radian)
    }

    private fun Pose2d.toVector(): Vector<N3> {
        return vec(`3`).fill(translation.x, translation.y, rotation.radians)
    }

    companion object {
        val kNominalDt = 0.01.second
    }
}