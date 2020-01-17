package frc.team4069.robot

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.commands.SubsystemHandler
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity

object Robot : TimedRobot() {

    override fun robotInit() {
        println("Robot init start")
        +Flywheel
        println("Robot init done")
    }

    override fun teleopInit() {
        println("teleop")
    }

    override fun autonomousInit() {
        println("init")
        Flywheel.enable()
        Flywheel.setReference(100.radian.velocity)
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    private operator fun SaturnSubsystem.unaryPlus() = SubsystemHandler.add(this)
}

fun main() {
    RobotBase.startRobot { Robot }
}
