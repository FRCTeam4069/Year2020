package frc.team4069.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.util.launchFrequency
import frc.team4069.saturn.lib.vision.LimelightCamera
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch

object Vision {
    private val limelight = LimelightCamera()

    private var numberOfEnables = 0
    var ledState: LimelightCamera.LEDState
        get() = limelight.ledState
        set(value) {
            if(value == LimelightCamera.LEDState.ForceOn) {
                numberOfEnables++
                limelight.ledState = value
            }else if(value == LimelightCamera.LEDState.ForceOff) {
                numberOfEnables--
                if(numberOfEnables < 0) {
                    numberOfEnables = 0
                }

                if(numberOfEnables == 0) {
                    limelight.ledState = value
                }
            } else {
                limelight.ledState = value
            }

        }

    val xOffset: SIUnit<Unitless> get() = limelight.xOffset.degree
    val targetArea: Double get() = limelight.targetArea

    val hasTarget: Boolean get() = limelight.hasTargets

    val cameraPose: Pose2d get() = limelight.cameraPose

    init {
        ledState = LimelightCamera.LEDState.ForceOff
    }
}