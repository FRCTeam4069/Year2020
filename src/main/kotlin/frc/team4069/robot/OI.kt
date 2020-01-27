package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.saturn.lib.hid.*
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.util.deadband

object OI {
    val controller = xboxController(0) {
        button(kA) {
            var set = false

            changeOn {
                if(!set) {
                    set = true
//                    Flywheel.setReference(Flywheel.TEST_SHOOTING_PRESET)
                    Flywheel.setDutyCycle(0.8)
                }else {
                    set = false
//                    Flywheel.setReference(0.radian.velocity)
                    Flywheel.setDutyCycle(0.0)
                }
            }
        }

        button(kBumperLeft) {
            changeOn {
                Hood.setPosition(0.0)
            }
        }

        button(kBumperRight) {
            changeOn {
                Hood.setPosition(1.0)
            }
        }

        button(kX) {
            changeOn {
                TowerOfDoom.setDutyCycle(0.8)
            }

            changeOff {
                TowerOfDoom.setDutyCycle(0.0)
            }
        }

        button(kB) {
            changeOn {
                TowerOfDoom.setDutyCycle(-0.4)
            }

            changeOff {
                TowerOfDoom.setDutyCycle(0.0)
            }
        }
    }

    val driveSpeed: Double
        get() = (controller.getTriggerAxis(GenericHID.Hand.kRight) - controller.getTriggerAxis(GenericHID.Hand.kLeft)).deadband(0.2)

    val driveTurn: Double
        get() = controller.getX(GenericHID.Hand.kLeft).deadband(0.2)
}