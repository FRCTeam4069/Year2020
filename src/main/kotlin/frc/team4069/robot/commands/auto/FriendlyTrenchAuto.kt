package frc.team4069.robot.commands.auto

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team4069.robot.Constants
import frc.team4069.robot.Trajectories
import frc.team4069.robot.commands.elevator.SetTowerSpeedCommand
import frc.team4069.robot.commands.elevator.UnloadTowerCommand
import frc.team4069.robot.commands.flywheel.AutoSetFlywheelReferenceCommand
import frc.team4069.robot.commands.flywheel.SetFlywheelReferenceCommand
import frc.team4069.robot.commands.intake.AdjustIntakePivotCommand
import frc.team4069.robot.commands.intake.SetIntakeSpeedCommand
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Intake
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.commands.parallel
import frc.team4069.saturn.lib.commands.parallelRace
import frc.team4069.saturn.lib.commands.sequential
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.subsystem.TrajectoryTrackerCommand

@Suppress("FunctionName")
fun FriendlyTrenchAuto() = sequential {
    +InstantCommand({ Drivetrain.resetGyro(90.degree) }, arrayOf(Drivetrain))
    +InstantCommand({ Flywheel.enable() }, arrayOf(Flywheel))
    +InstantCommand({ TowerOfDoom.ballCount.set(3) }, arrayOf(TowerOfDoom))
    +parallelRace {
        +AutoSetFlywheelReferenceCommand()
        +sequential {
            +TrajectoryTrackerCommand(
                Drivetrain,
                Constants.RAMSETE_B,
                Constants.RAMSETE_ZETA,
                trajectory = Trajectories.friendlyWallToShoot,
                leftPid = Drivetrain.leftPid,
                rightPid = Drivetrain.rightPid,
                feedforward = Drivetrain.feedforward,
                resetPose = true
            )
            +WaitUntilCommand { Flywheel.controller.atGoal }
            +SetTowerSpeedCommand(0.65)
            +WaitCommand(0.55)
            +SetTowerSpeedCommand(0.0)
        }
    }
    +AdjustIntakePivotCommand(Intake.PivotPosition.Extended)
    +SetIntakeSpeedCommand(0.8)
    +SetTowerSpeedCommand(0.2)
    +TrajectoryTrackerCommand(
        Drivetrain,
        Constants.RAMSETE_B,
        Constants.RAMSETE_ZETA,
        trajectory = Trajectories.shootPositionToTrench,
        leftPid = Drivetrain.leftPid,
        rightPid = Drivetrain.rightPid,
        feedforward = Drivetrain.feedforward
    )
    +SetIntakeSpeedCommand(0.0)
    +SetTowerSpeedCommand(0.0)
    +parallel {
        +AutoSetFlywheelReferenceCommand()
        +sequential {
            +TrajectoryTrackerCommand(
                Drivetrain,
                Constants.RAMSETE_B,
                Constants.RAMSETE_ZETA,
                trajectory = Trajectories.trenchToShootPosition,
                leftPid = Drivetrain.leftPid,
                rightPid = Drivetrain.rightPid,
                feedforward = Drivetrain.feedforward
            )
            +WaitUntilCommand { Flywheel.controller.atGoal }
            +SetIntakeSpeedCommand(1.0)
            +SetTowerSpeedCommand(0.65)
        }
    }
}
