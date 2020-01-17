package frc.team4069.robot

import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.FlywheelCharacterizationCommand
import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.shuffleboard.logging.tab
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.zeromq.SocketType
import org.zeromq.ZContext
import kotlin.concurrent.thread

object Robot : SaturnRobot() {

    override fun robotInit() {
        +Flywheel
    }

    override fun autonomousInit() {
        Flywheel.enable()
        Flywheel.setReference(350.radian.velocity)
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}

fun main() {
    Robot.start()
}
