package frc.team4069.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.music.Orchestra
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.robot.subsystems.drivetrain.Drivetrain
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch

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
//        OperatorDriveCommand().schedule()
//        Hood.setPosition(0.75)
    }

    override fun robotPeriodic() {
        controls.forEach(SaturnHID<*>::update)
    }

    override fun autonomousInit() {
//        Flywheel.enable()
//        Flywheel.setReference(300.radian.velocity)
        Hood.setPosition(0.7)
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
