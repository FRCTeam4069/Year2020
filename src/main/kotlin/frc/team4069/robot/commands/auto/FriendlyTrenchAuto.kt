package frc.team4069.robot.commands.auto

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
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
                trajectory = Trajectories.blueFriendlyWallToShoot,
                leftPid = Drivetrain.leftPid,
                rightPid = Drivetrain.rightPid,
                feedforward = Drivetrain.feedforward,
                resetPose = true
            )
            +WaitCommand(0.75)
            +UnloadTowerCommand()
        }
    }
//    +AdjustIntakePivotCommand(Intake.PivotPosition.Extended)
    +SetIntakeSpeedCommand(1.0)
    +TrajectoryTrackerCommand(
        Drivetrain,
        Constants.RAMSETE_B,
        Constants.RAMSETE_ZETA,
        trajectory = Trajectories.shootPositionToGenerator,
        leftPid = Drivetrain.leftPid,
        rightPid = Drivetrain.rightPid,
        feedforward = Drivetrain.feedforward
    )
    +WaitCommand(0.75)
    +SetIntakeSpeedCommand(0.0)
    +parallelRace {
        +AutoSetFlywheelReferenceCommand()
        +sequential {
            +TrajectoryTrackerCommand(
                Drivetrain,
                Constants.RAMSETE_B,
                Constants.RAMSETE_ZETA,
                trajectory = Trajectories.generatorToShootPosition,
                leftPid = Drivetrain.leftPid,
                rightPid = Drivetrain.rightPid,
                feedforward = Drivetrain.feedforward
            )
            +WaitCommand(0.5)
            +SetTowerSpeedCommand(0.35)
            +SetIntakeSpeedCommand(1.0)
        }
    }
}

