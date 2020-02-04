package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpiutil.math.numbers.N2
import edu.wpi.first.wpiutil.math.numbers.N3
import edu.wpi.first.wpiutil.math.numbers.N5
import frc.team4069.robot.subsystems.drivetrain.DrivetrainEstimator
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.matrix.Vector
import frc.team4069.saturn.lib.mathematics.model.gearbox
import frc.team4069.saturn.lib.mathematics.model.kMotorCim
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Rotation2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Twist2d
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.TrajectoryConfig
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.radian
import org.junit.Assert
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import java.util.*
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

class ExtendedKalmanFilterTest {

    fun dynamics(x: Vector<N5>, u: Vector<N2>): Vector<N5> {
        val motors = gearbox(kMotorCim, 2)

        val Ghigh = 7.08
        val rb = 0.8382.meter / 2.0
        val r = 0.0746124.meter
        val m = 63.503.kilo.gram
        val J = SIUnit<Mult<Kilogram, Mult<Meter, Meter>>>(5.6) // Moment of inertia

        val C1 = -Ghigh.pow(2) * motors.Kt.value / (motors.Kv.value * motors.R.value * r.value.pow(2))
        val C2 = Ghigh * motors.Kt.value / (motors.R.value * r.value)
        val k1 = (1 / m.value + rb.value.pow(2) / J.value)
        val k2 = (1 / m.value - rb.value.pow(2) / J.value)

        val vl = x[3].meter.velocity
        val vr = x[4].meter.velocity

        val Vl = u[0].volt
        val Vr = u[1].volt

        val result = zeros(`5`)
        val v = 0.5 * (vl + vr)
        result[0] = v.value * cos(x[2])
        result[1] = v.value * sin(x[2])
        result[2] = ((vr - vl) / (2.0 * rb)).value
        result[3] =
            k1 * ((C1 * vl.value) + (C2 * Vl.value)) +
                    k2 * ((C1 * vr.value) + (C2 * Vr.value))
        result[4] =
            k2 * ((C1 * vl.value) + (C2 * Vl.value)) +
                    k1 * ((C1 * vr.value) + (C2 * Vr.value))

        return result
    }

    fun localMeasurementModel(x: Vector<N5>, u: Vector<N2>): Vector<N3> {
        return vec(`3`).fill(x[2], x[3], x[4])
    }

    fun globalMeasurementModel(x: Vector<N5>, u: Vector<N2>): Vector<N5> {
        return x
    }

    @Test
    fun extendedKalmanFilterTest() {
        val dt = 0.00505.second

        val observer = ExtendedKalmanFilter(
            `5`, `2`, `3`, this::dynamics, this::localMeasurementModel,
            vec(`5`).fill(0.5, 0.5, 10.0, 1.0, 1.0),
            vec(`3`).fill(0.0001, 0.01, 0.01), dt
        )

        val u = vec(`2`).fill(12.0, 12.0)

        observer.predict(u, dt)

        val localY = localMeasurementModel(observer.xHat, u)
        observer.correct(u, localY)

        val globalY = globalMeasurementModel(observer.xHat, u)
        val R = StateSpaceUtils.makeCovMatrix({ 5 }, vec(`5`).fill(0.01, 0.01, 0.0001, 0.01, 0.01))
        observer.correct(`5`, u, globalY, this::globalMeasurementModel, R)
    }

    private fun pexp(twist: Vector<N3>): Vector<N3> {
        val twist = Twist2d(twist[0].meter, twist[1].meter, twist[2].radian)

        val pose = Pose2d().exp(twist)
        return vec(`3`).fill(pose.translation.x, pose.translation.y, pose.rotation.radians)
    }

    @Test
    fun testEkf2() {
        val dt = 0.01.second
        val observer = DrivetrainEstimator()
        val odometry = DifferentialDriveOdometry(Rotation2d(0.radian))
        odometry.update(Rotation2d(45.degree), 0.02 / sqrt(2.0), 0.02)

        observer.update(dt, vec(`3`).fill(0.02 / sqrt(2.0), 0.02, 45.degree.radian))

        Assert.assertEquals(observer.observer.xHat[0], odometry.poseMeters.translation.x, 1E-9)
        Assert.assertEquals(observer.observer.xHat[1], odometry.poseMeters.translation.y, 1E-9)
        Assert.assertEquals(observer.observer.xHat[2], odometry.poseMeters.rotation.radians, 1E-9)
    }


    @Test
    fun testEkfCorrect() {
        val observer = DrivetrainEstimator()
        val dt = DrivetrainEstimator.kNominalDt

        val traj = TrajectoryGenerator.generateTrajectory(
            listOf(
                Pose2d(),
                Pose2d(5.meter, 5.meter, 0.radian)
            ),
            TrajectoryConfig(
                3.meter.velocity,
                3.meter.acceleration,
                listOf(),
                0.meter.velocity,
                0.meter.velocity,
                reversed = false
            )
        )

        val kinematics = DifferentialDriveKinematics(observer.trackWidth)
        var lastPose: Pose2d? = null

        val trajXs = mutableListOf<Double>()
        val trajYs = mutableListOf<Double>()
        val measuredXs = mutableListOf<Double>()
        val measuredYs = mutableListOf<Double>()
        val filterXs = mutableListOf<Double>()
        val filterYs = mutableListOf<Double>()
        val rand = Random()

        var t = 0.0
        while(t <= traj.totalTimeSeconds) {
            val state = traj.sample(t)
            val wheelSpeeds = kinematics.toWheelSpeeds(
                ChassisSpeeds(
                    state.velocityMetersPerSecond,
                    0.0,
                    state.velocityMetersPerSecond * state.curvatureRadPerMeter
                )
            )

            val u = if (lastPose != null) {
                vec(`3`).fill(
                    wheelSpeeds.leftMetersPerSecond * dt.value, wheelSpeeds.rightMetersPerSecond * dt.value,
                    state.poseMeters.rotation.radians - lastPose.rotation.radians)
            } else {
                vec(`3`).fill(
                    wheelSpeeds.leftMetersPerSecond * dt.value,
                    wheelSpeeds.rightMetersPerSecond * dt.value,
                    0.0
                )
            }

            val measuredPose = state.poseMeters + Pose2d(rand.nextGaussian() * 0.05, rand.nextGaussian() * 0.1, Rotation2d(rand.nextGaussian().radian * 0.01))

            observer.update(dt, u, measuredPose)
            trajXs.add(state.poseMeters.translation.x)
            trajYs.add(state.poseMeters.translation.y)
            filterXs.add(observer.pose.translation.x)
            filterYs.add(observer.pose.translation.y)
            measuredXs.add(measuredPose.translation.x)
            measuredYs.add(measuredPose.translation.y)
            lastPose = state.poseMeters
            t += dt.value
        }

        var chart = XYChartBuilder().build()
        chart.addSeries("Trajectory", trajXs, trajYs)
        chart.addSeries("EKF", filterXs, filterYs)
//        chart.addSeries("Measurements", measuredXs, measuredYs)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000000000)
    }

    operator fun Pose2d.plus(other: Pose2d) = Pose2d(
        translation.x + other.translation.x,
        translation.y + other.translation.y,
        rotation + other.rotation
    )
}
