package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU

object Constants {
    const val FLYWHEEL_J = 0.00157829716 // kg m2

    val kLeftDrivetrainUnitModel = NativeUnitLengthModel(1.STU, 3.534.inch)
    val kRightDrivetrainUnitModel = NativeUnitLengthModel(1.STU, 3.684.inch)
}