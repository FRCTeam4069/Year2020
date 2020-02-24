package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.mathematics.units.derived.AccelerationFeedforward
import frc.team4069.saturn.lib.mathematics.units.derived.VelocityFeedforward
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.mathematics.units.volt

object Constants {
    val kLeftDrivetrainUnitModel = NativeUnitLengthModel(2048.STU, 1.73.inch)
    val kRightDrivetrainUnitModel = NativeUnitLengthModel(2048.STU, 1.73.inch)

    val DRIVETRAIN_KV = SIUnit<VelocityFeedforward>(2.15)
    val DRIVETRAIN_KA = SIUnit<AccelerationFeedforward>(0.4)
    val DRIVETRAIN_KS = 0.176.volt

    const val RAMSETE_B = 2.0
    const val RAMSETE_ZETA = 0.7
}