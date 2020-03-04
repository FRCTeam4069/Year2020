package frc.team4069.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.TrajectoryConfig
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.LinearAcceleration
import frc.team4069.saturn.lib.mathematics.units.conversions.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.conversions.feet

object Trajectories {

    val kDefaultConfig = TrajectoryConfig(
        maxVelocity = 7.feet.velocity,
        maxAcceleration = 5.feet.acceleration,
        constraints = listOf(CentripetalAccelerationConstraint(3.feet.acceleration.value)),
        startVelocity = 0.feet.velocity,
        endVelocity = 0.feet.velocity,
        reversed = false
    )

    val blueFriendlyWallToShoot = waypoints(
        Pose2d(11.878.feet, 25.388.feet, 90.degree),
        Pose2d(9.873.feet, 23.138.feet, 27.degree)
    ).generateTrajectory(
        "Friendly wall to Shoot",
        kDefaultConfig.copy(
            maxVelocity = 3.feet.velocity,
            maxAcceleration = 2.feet.acceleration,
            constraints = listOf(
                DifferentialDriveVoltageConstraint(Drivetrain.feedforward, Drivetrain.kinematics, 12.0)
            ),
            reversed = true
        )
    )

    val shootPositionToGenerator = waypoints(
        Pose2d(9.873.feet, 23.138.feet, 27.degree),
        Pose2d(14.095.feet, 14.459.feet, -90.degree),
        Pose2d(18.513.feet, 13.087.feet, 34.degree)
    ).generateTrajectory(
        "Shoot Position to Generator",
        kDefaultConfig.copy(
            maxVelocity = 7.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                DifferentialDriveVoltageConstraint(Drivetrain.feedforward, Drivetrain.kinematics, 12.0),
                CentripetalAccelerationConstraint(3.feet.acceleration.value),
                VelocityLimitRegionConstraint(
                    Rectangle2d(
                        16.meter..21.meter,
                        12.meter..16.meter
                    ),
                    4.feet.velocity
                )
            )
        )
    )

    val generatorToShootPosition = waypoints(
        Pose2d(18.513.feet, 13.087.feet, 34.degree),
        Pose2d(14.095.feet, 14.459.feet, -90.degree),
        Pose2d(9.873.feet, 23.138.feet, 27.degree)
    ).generateTrajectory(
        "Generator to Shoot Position",
        kDefaultConfig.copy(
            maxVelocity = 7.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                DifferentialDriveVoltageConstraint(Drivetrain.feedforward, Drivetrain.kinematics, 12.0),
                CentripetalAccelerationConstraint(3.feet.acceleration.value)
            ),
            reversed = true
        )
    )

    val blueFriendlyWallToGoal = waypoints(
        Pose2d(11.878.feet, 25.388.feet, 90.degree),
        Pose2d(1.997.feet, 19.feet, 0.degree)
    ).generateTrajectory(
        "Friendly Wall To Goal",
        config = kDefaultConfig.copy(reversed = true))

    val blueGoalToGeneratorSwitch = waypoints(
        Pose2d(1.997.feet, 19.feet, 0.degree),
        Pose2d(10.862.feet, 15.474.feet, -37.degree),
        Pose2d(18.865.feet, 12.673.feet, 0.degree)
    ).generateTrajectory(
        "Blue Goal to Generator Switch",
        config = kDefaultConfig.copy(
            maxVelocity = 16.feet.velocity,
            maxAcceleration = 10.feet.acceleration,
            constraints = listOf(
                CentripetalAccelerationConstraint(6.feet.acceleration.value)
            )
        )
    )

    val generatorSwitchToBlueGoal = waypoints(
        Pose2d(18.865.feet, 12.673.feet, 0.degree),
        Pose2d(10.862.feet, 15.474.feet, -37.degree),
        Pose2d(1.997.feet, 19.feet, 0.degree)
    ).generateTrajectory(
        "Generator Switch to Blue Goal",
        config = kDefaultConfig.copy(
            maxVelocity = 16.feet.velocity,
            maxAcceleration = 10.feet.acceleration,
            constraints = listOf(CentripetalAccelerationConstraint(6.feet.acceleration.value)),
            reversed = true
        )
    )

    val enemyWallToTrench = waypoints(
        Pose2d(9.591.feet, 1.372.feet, 90.degree),
        Pose2d(13.479.feet, 8.352.feet, -38.degree),
        Pose2d(20.404.feet, 2.59.feet, -30.degree)
    ).generateTrajectory(
        "Enemy Wall to Trench",
        kDefaultConfig.copy(
        maxVelocity = 7.feet.velocity,
        maxAcceleration = 4.5.feet.acceleration,
        constraints = listOf(
            CentripetalAccelerationConstraint(2.5.feet.acceleration.value),
            DifferentialDriveVoltageConstraint(Drivetrain.feedforward, Drivetrain.kinematics, 12.0),
            VelocityLimitRegionConstraint(
                Rectangle2d(18.meter..22.meter, 0.meter..6.meter),
                3.feet.velocity
            )
        )
    ))

    val enemyTrenchToShoot = waypoints(
        Pose2d(20.404.feet, 2.59.feet, -30.degree),
        Pose2d(4.354.feet, 12.582.feet, -39.degree),
        Pose2d(1.847.feet, 18.781.feet, 0.degree)
    ).generateTrajectory(
        "Enemy Trench to Shooting Position",
        kDefaultConfig.copy(
            maxVelocity = 12.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                CentripetalAccelerationConstraint(3.feet.acceleration.value),
                DifferentialDriveVoltageConstraint(Drivetrain.feedforward, Drivetrain.kinematics, 12.0)
            ),
            reversed = true
        )
    )

    val enemyTrenchToShootBack = waypoints(
        Pose2d(20.404.feet, 2.59.feet, -30.degree),
        Pose2d(15.479.feet, 6.704.feet, -54.degree),
        Pose2d(10.feet, 15.feet, -15.degree)
    ).generateTrajectory("Enemy Trench to Farther Shot",
        kDefaultConfig.copy(
            maxVelocity = 10.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                CentripetalAccelerationConstraint(3.feet.acceleration.value),
                DifferentialDriveVoltageConstraint(Drivetrain.feedforward, Drivetrain.kinematics, 12.0)
            ),
            reversed = true
        ))

    private fun waypoints(vararg poses: Pose2d) = listOf(*poses)

    private fun List<Pose2d>.generateTrajectory(
        name: String,
        config: TrajectoryConfig = kDefaultConfig,
        clampedCubic: Boolean = false
    ): Trajectory {
        println("Generating trajectory $name.")
        return if(clampedCubic) {
            val first = this.first()
            val last = this.last()
            val innerWaypoints = this.subList(1, lastIndex - 1).map { it.translation }
            return TrajectoryGenerator.generateTrajectory(first, innerWaypoints, last, config)
        } else TrajectoryGenerator.generateTrajectory(this, config)
    }

    private fun TrajectoryConfig.copy(
        maxVelocity: SIUnit<LinearVelocity> = this.maxVelocity.meter.velocity,
        maxAcceleration: SIUnit<LinearAcceleration> = this.maxAcceleration.meter.acceleration,
        constraints: List<TrajectoryConstraint> = this.constraints,
        startVelocity: SIUnit<LinearVelocity> = this.startVelocity.meter.velocity,
        endVelocity: SIUnit<LinearVelocity> = this.endVelocity.meter.velocity,
        reversed: Boolean = this.isReversed
    ) = TrajectoryConfig(
        maxVelocity = maxVelocity,
        maxAcceleration = maxAcceleration,
        constraints = constraints,
        startVelocity = startVelocity,
        endVelocity = endVelocity,
        reversed = reversed
    )
}