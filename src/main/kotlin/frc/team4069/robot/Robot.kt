package frc.team4069.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.OperatorDriveCommand
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.second
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.util.DeltaTime

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
//        Flywheel.enable()
        OperatorDriveCommand().schedule()
//        Hood.setPosition(0.75)
    }

    override fun robotPeriodic() {
        controls.forEach(SaturnHID<*>::update)
    }

    override fun autonomousInit() {
        Flywheel.enable()
        Flywheel.setReference(150.radian.velocity)
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
