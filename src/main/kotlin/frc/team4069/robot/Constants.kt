package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.mathematics.units.derived.AccelerationFeedforward
import frc.team4069.saturn.lib.mathematics.units.derived.VelocityFeedforward
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU

object Constants {
    val kLeftDrivetrainUnitModel = NativeUnitLengthModel(2048.STU, 1.73.inch)
    val kRightDrivetrainUnitModel = NativeUnitLengthModel(2048.STU, 1.73.inch)

    val DRIVETRAIN_KV = SIUnit<VelocityFeedforward>(2.34)
    val DRIVETRAIN_KA = SIUnit<AccelerationFeedforward>(0.4)
    val DRIVETRAIN_KS = 0.176.volt

    const val RAMSETE_B = 2.7
    const val RAMSETE_ZETA = 0.99

    val FLYWHEEL_SPD_M = SIUnit<Fraction<AngularVelocity, Meter>>(27.924)
    val FLYWHEEL_SPD_B = 647.111.rpm
}