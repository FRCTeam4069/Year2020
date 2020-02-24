package frc.team4069.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.TrajectoryConfig
import frc.team4069.saturn.lib.mathematics.units.acceleration
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity

object Trajectories {

    val kDefaultConfig = TrajectoryConfig(
        maxVelocity = 2.feet.velocity,
        maxAcceleration = 1.feet.acceleration,
        constraints = listOf(CentripetalAccelerationConstraint(0.5.feet.acceleration.value)),
        startVelocity = 0.feet.velocity,
        endVelocity = 0.feet.velocity,
        reversed = false
    )

    val testTrajectory = waypoints(
        Pose2d(2.294.feet, 14.feet, 0.radian),
        Pose2d(11.015.feet, 9.201.feet, 0.radian)
    ).generateTrajectory()


    private fun waypoints(vararg poses: Pose2d) = listOf(*poses)

    private fun List<Pose2d>.generateTrajectory(
        config: TrajectoryConfig = kDefaultConfig
    ): Trajectory = TrajectoryGenerator.generateTrajectory(this, config)
}