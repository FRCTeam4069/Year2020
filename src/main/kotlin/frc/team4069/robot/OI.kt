package frc.team4069.robot

import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.robot.subsystem.TowerOfDoom
import frc.team4069.saturn.lib.hid.*
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity

object OI {
    val controller = xboxController(0) {
        button(kA) {
            var set = false

            changeOn {
                if(!set) {
                    set = true
                    Flywheel.setReference(Flywheel.TEST_SHOOTING_PRESET)
                }else {
                    set = false
                    Flywheel.setReference(0.radian.velocity)
                }
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
}