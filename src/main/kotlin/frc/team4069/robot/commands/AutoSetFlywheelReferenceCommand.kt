package frc.team4069.robot.commands

import frc.team4069.robot.Constants
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.Vision
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.twodim.geometry.xU
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.vision.LimelightCamera

class AutoSetFlywheelReferenceCommand : SaturnCommand(Flywheel, Hood) {

    override fun initialize() {
        Hood.setPosition(0.5)
        Vision.ledState = LimelightCamera.LEDState.ForceOn
        Flywheel.enable()
    }

    var higherHood = false

    override fun execute() {
        val dist = -Vision.cameraPose.translation.xU

        val spd = when {
            dist > 70.inch && dist < 89.inch -> {
                if(higherHood) {
                    higherHood = false
                    Hood.setPosition(0.5)
                }
                1.8 * (Constants.FLYWHEEL_SPD_M_075_HOOD * dist + Constants.FLYWHEEL_SPD_B_075_HOOD)
            }
            dist > 89.inch -> {
                if(!higherHood) {
                    higherHood = true
                    Hood.setPosition(0.75)
                }
                1.7 * (Constants.FLYWHEEL_SPD_M_075_HOOD * dist + Constants.FLYWHEEL_SPD_B_075_HOOD)
            }
            else -> {
                if(higherHood) {
                    higherHood = false
                    Hood.setPosition(0.5)
                }
                2 * (Constants.FLYWHEEL_SPD_M_05_HOOD * dist + Constants.FLYWHEEL_SPD_B_05_HOOD)
            }
        }

        Flywheel.setReference(spd)
    }

    override fun end(interrupted: Boolean) {
        Hood.setPosition(0.0)
        Flywheel.disable()
        Flywheel.setReference(0.rpm)
        Vision.ledState = LimelightCamera.LEDState.ForceOff
    }
}