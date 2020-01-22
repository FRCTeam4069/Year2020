package frc.team4069.robot.subsystem

import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.conversions.degree
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitRotationModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX

object Hood : SaturnSubsystem() {

    private val talon = SaturnSRX(2, NativeUnitRotationModel(4096.STU))
    private val encoder = talon.encoder
    private val maxPos = 45.degree * 2.33

    init {
        talon.talon.apply {
            configPeakOutputForward(0.3)
            configPeakOutputReverse(-0.3)
            config_kP(0, 1.0)
            config_kI(0, 0.001)
            config_kD(0, 0.0)
            selectedSensorPosition = 0
        }
    }

    // position is 0 to 1, where 0 is fully retracted and 1 is fully extended
    fun setPosition(position: Double) {
        val setpoint = maxPos * position
        talon.setPosition(setpoint)
//        talon.set(ControlMode.PercentOutput, 0.1)
        println(encoder.position.degree / 2.33)
    }

}