package frc.team4069.robot.commands.flywheel

import frc.team4069.robot.Constants
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.Vision
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.twodim.geometry.xU
import frc.team4069.saturn.lib.mathematics.twodim.geometry.yU
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
        val dist = sqrt(Vision.cameraPose.translation.xU.pow2() + Vision.cameraPose.translation.yU.pow2())
	println("Distance: ${dist.inch}")
        val magicMultiplier = 1.7
        if(!higherHood) {
            higherHood = true
            Hood.setPosition(0.5)
        }
        val spd = magicMultiplier * (Constants.FLYWHEEL_SPD_M_05_HOOD * dist + Constants.FLYWHEEL_SPD_B_05_HOOD)
        Flywheel.setReference(spd)
    }

    override fun end(interrupted: Boolean) {
        Hood.setPosition(0.0)
        Flywheel.disable()
        Flywheel.setReference(0.rpm)
        Vision.ledState = LimelightCamera.LEDState.ForceOff
    }
}
