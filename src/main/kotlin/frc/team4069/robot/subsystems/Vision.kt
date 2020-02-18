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

    val xOffset: SIUnit<Unitless> get() = limelight.xOffset.degree

    val hasTarget: Boolean get() = limelight.hasTargets
}