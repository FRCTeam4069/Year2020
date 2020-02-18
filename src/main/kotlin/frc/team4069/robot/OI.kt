package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.robot.commands.CenterToTapeCommand
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.hid.*
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.rpm
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.util.deadband

object OI {
    val controller = xboxController(0) {

        button(kBumperRight) {
            val command = CenterToTapeCommand()
            changeOn { command.schedule() }
            changeOff { command.cancel() }
        }

        button(kA) {
            changeOn {
                Drivetrain.gear = Drivetrain.Gear.Low
            }

            changeOff {
                Drivetrain.gear = Drivetrain.Gear.High
            }
        }

        button(kStart) {
            changeOn {
                Drivetrain.gear = Drivetrain.Gear.Low
            }
        }

        button(kX) {
            changeOn {
                TowerOfDoom.setTowerDutyCycle(0.65)

            }

            changeOff {
                TowerOfDoom.setTowerDutyCycle(0.0)
            }
        }

        button(kB) {
            changeOn {
                TowerOfDoom.setTowerDutyCycle(-0.4)
            }

            changeOff {
                TowerOfDoom.setTowerDutyCycle(0.0)
            }
        }
    }

    val operatorController = xboxController(1) {

        button(kA) {
            var set = false

            changeOn {
                if (!set) {
                    set = true
//                    Flywheel.setReference(1675.rpm)
                    Flywheel.setReference(Flywheel.TRENCH_SHOT_PRESET)
                } else {
                    set = false
                    Flywheel.setReference(0.radian.velocity)
                }
            }
        }
    }

    val driveSpeed: Double
        get() = (controller.getTriggerAxis(GenericHID.Hand.kRight) - controller.getTriggerAxis(GenericHID.Hand.kLeft))

    val driveTurn: Double
        get() = controller.getX(GenericHID.Hand.kLeft).deadband(0.2)

    val climberSpeed: Double
        get() = controller.getY(GenericHID.Hand.kRight).deadband(0.2)

    val intakeSpeed: Double
        get() = (operatorController.getTriggerAxis(GenericHID.Hand.kRight) - operatorController.getTriggerAxis(
            GenericHID.Hand.kLeft
        )).deadband(0.1)

    val towerSpeed: Double
        get() = operatorController.getY(GenericHID.Hand.kLeft)

    val hoodSpeed: Double
        get() = operatorController.getY(GenericHID.Hand.kRight)
}
