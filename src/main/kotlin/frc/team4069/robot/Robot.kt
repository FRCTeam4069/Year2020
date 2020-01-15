package frc.team4069.robot

import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.FlywheelCharacterizationCommand
import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.shuffleboard.logging.tab

object Robot : SaturnRobot() {

    override fun robotInit() {
        +Flywheel

        tab("Robot") {
            list("Flywheel") {

                size(4, 4)

                textView("KF Error", { Flywheel.controller.velocity.value - Flywheel.velocity.value }) {
                    size(2, 1)
                    position(0, 0)
                }

                textView("Velocity", { Flywheel.velocity.value }) {
                    size(2, 1)
                    position(0, 3)
                }

                textView("Error (rad s^-1)", { 350 - Flywheel.controller.measuredVelocity.value }) {
                    size(2, 1)
                    position(0, 1)
                }

                textView("Voltage", { Flywheel.controller.voltage.value }) {
                    size(2, 1)
                    position(0, 2)
                }
            }
        }
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
