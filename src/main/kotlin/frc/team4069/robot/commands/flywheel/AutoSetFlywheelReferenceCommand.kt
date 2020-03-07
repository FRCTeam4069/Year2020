package frc.team4069.robot.commands.flywheel

import edu.wpi.first.wpilibj.MedianFilter
import frc.team4069.robot.Constants
import frc.team4069.robot.subsystems.Hood
import frc.team4069.robot.subsystems.Vision
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.robot.util.InterpolatableDouble
import frc.team4069.robot.util.InterpolatableTreeMap
import frc.team4069.robot.util.interpolatableMapOf
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.twodim.geometry.xU
import frc.team4069.saturn.lib.mathematics.twodim.geometry.yU
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.vision.LimelightCamera

class AutoSetFlywheelReferenceCommand : SaturnCommand(Flywheel, Hood) {

    val xFilter = MedianFilter(20)
    val yFilter = MedianFilter(20)
    private val distancesLUT05 = interpolatableMapOf(
        71.0 to InterpolatableDouble(1.9),
        83.0 to InterpolatableDouble(1.7),
        88.0 to InterpolatableDouble(1.65)
    )
    private val distancesLUT075 = interpolatableMapOf(
        88.0 to InterpolatableDouble(1.9),
        97.0 to InterpolatableDouble(1.7),
        107.0 to InterpolatableDouble(1.67),
        118.0 to InterpolatableDouble(1.59),
        121.4 to InterpolatableDouble(1.65),
        128.0 to InterpolatableDouble(1.6),
        132.5 to InterpolatableDouble(1.63)
    )

    override fun initialize() {
        Hood.setPosition(0.5)
        Vision.ledState = LimelightCamera.LEDState.ForceOn
        Flywheel.enable()

    }

    fun getMagicMultiplierAngle05(distance: Double) : Double {
//        val distances = arrayOf(71, 83, 88)
//        val multipliers = arrayOf(1.9, 1.7, 1.65)
//        for(i in 1 until distances.size) {
//            if (distances[i] > distance) {
//                val p = (distance - distances[i - 1]) / (distances[i] - distances[i - 1])
//                return multipliers[i - 1] + (p * (multipliers[i] - multipliers[i - 1]))
//            }
//        }
//        return -1.0
        return distancesLUT05[distance]?.value ?: Double.NaN
    }

    fun getMagicMultiplierAngle075(distance: Double) : Double {
//        val distances = arrayOf(88, 97, 107, 118)
//        val multipliers = arrayOf(1.9, 1.7, 1.67, 1.59)
//        for(i in 1 until distances.size) {
//            if (distances[i] > distance) {
//                val p = (distance - distances[i - 1]) / (distances[i] - distances[i - 1])
//                return multipliers[i - 1] + (p * (multipliers[i] - multipliers[i - 1]))
//            }
//        }
//        return -1.0
        return distancesLUT075[distance]?.value ?: Double.NaN
    }

    var higherHood = false

    override fun execute() {
        val pose = Vision.cameraPose
        val xMed = xFilter.calculate(pose.translation.xU.inch).inch
        val yMed = yFilter.calculate(pose.translation.yU.inch).inch
        val dist = sqrt(xMed.pow2() + yMed.pow2())
        println(dist.inch)
        val spd = when {
            dist > 88.inch -> {
                if(!higherHood) {
                    higherHood = true
                    Hood.setPosition(0.75)
                }
                getMagicMultiplierAngle075(dist.inch) * (Constants.FLYWHEEL_SPD_M_075_HOOD * dist + Constants.FLYWHEEL_SPD_B_075_HOOD)
            }
            else -> {
                if(higherHood) {
                    higherHood = false
                    Hood.setPosition(0.5)
                }
                getMagicMultiplierAngle05(dist.inch) * (Constants.FLYWHEEL_SPD_M_05_HOOD * dist + Constants.FLYWHEEL_SPD_B_05_HOOD)
            }
        }
        if(!spd.value.isNaN()) {
            Flywheel.setReference(spd)
        }
    }

    override fun end(interrupted: Boolean) {
        Hood.setPosition(0.0)
        Flywheel.disable()
        Flywheel.setReference(0.rpm)
        xFilter.reset()
        yFilter.reset()
        Vision.ledState = LimelightCamera.LEDState.ForceOff
    }
}
