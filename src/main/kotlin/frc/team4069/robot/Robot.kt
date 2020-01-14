package frc.team4069.robot

import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.FlywheelCharacterizationCommand
import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.shuffleboard.logging.tab

object Robot : SaturnRobot() {

    override fun robotInit() {
    }

    override fun autonomousInit() {
        FlywheelCharacterizationCommand().schedule()
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}

fun main() {
    Robot.start()
}
