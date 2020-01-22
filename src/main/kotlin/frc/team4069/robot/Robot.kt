package frc.team4069.robot

import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.OperatorDriveCommand
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID

object Robot : SaturnRobot() {

    private val controls = mutableListOf<SaturnHID<*>>()

    override fun robotInit() {
        +Drivetrain
        +Flywheel
        +TowerOfDoom
        +Hood
        +OI.controller
    }

    override fun teleopInit() {
        Flywheel.enable()
        OperatorDriveCommand().schedule()
    }

    override fun robotPeriodic() {
        controls.forEach(SaturnHID<*>::update)
    }

    override fun autonomousInit() {
        // Flywheel.enable()
    }

    override fun autonomousPeriodic() {
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    operator fun SaturnHID<*>.unaryPlus() {
        controls += this
    }
}

fun main() {
    Robot.start()
}
