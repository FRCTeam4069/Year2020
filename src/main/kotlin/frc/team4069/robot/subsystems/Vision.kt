package frc.team4069.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.hertz
import frc.team4069.saturn.lib.mathematics.units.meter
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.util.launchFrequency
import frc.team4069.saturn.lib.vision.LimelightCamera
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch

object Vision {
    private val limelight = LimelightCamera()

    private val filterX = MedianFilter(15)
    private val filterY = MedianFilter(15)
    private val filterTheta = MedianFilter(15)

    private var medianX = 0.0
    private var medianY = 0.0
    private var medianTheta = 0.0


    /**
     * If there is a vision target in sight, return the pose of the camera relative to the target.
     */
    val cameraRelativePose: Pose2d?
        get() {
            if(!limelight.hasTargets) return null

            val relativePose = Pose2d(medianX.meter, medianY.meter, medianTheta.radian)
            return fieldTargetPose.relativeTo(relativePose)
        }

    init {
        GlobalScope.launchFrequency(100.hertz) {
            val pose = limelight.cameraPose
            if(pose != Pose2d()) {
                medianX = filterX.calculate(pose.translation.x)
                medianY = filterY.calculate(pose.translation.y)
                medianTheta = filterTheta.calculate(pose.rotation.radians)
            }
        }
    }

    private val fieldTargetPose = if(DriverStation.getInstance().alliance == DriverStation.Alliance.Blue) {
        Pose2d(y = 19.feet)
    }else {
        Pose2d(x = 54.feet, y = 8.feet)
    }
}