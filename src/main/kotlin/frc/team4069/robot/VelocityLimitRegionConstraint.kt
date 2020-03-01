package frc.team4069.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.LinearVelocity

class VelocityLimitRegionConstraint(val region: Rectangle2d, val velocityLimit: SIUnit<LinearVelocity>) : TrajectoryConstraint {
    override fun getMaxVelocityMetersPerSecond(
        poseMeters: Pose2d,
        curvatureRadPerMeter: Double,
        velocityMetersPerSecond: Double
    ): Double {
        return if(poseMeters.translation in region) velocityLimit.value else Double.POSITIVE_INFINITY
    }

    override fun getMinMaxAccelerationMetersPerSecondSq(
        poseMeters: Pose2d,
        curvatureRadPerMeter: Double,
        velocityMetersPerSecond: Double
    ): TrajectoryConstraint.MinMax {
        return TrajectoryConstraint.MinMax()
    }

}