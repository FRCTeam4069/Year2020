package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.conversions.degree
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitRotationModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import frc.team4069.saturn.lib.shuffleboard.logging.tab

object Hood : SaturnSubsystem() {

    private val talon = SaturnSRX(RobotMap.Hood.TALON_ID, NativeUnitRotationModel(4096.STU))
    private val encoder = talon.encoder
    private val maxPos = 40.degree * 2.33 // After additional reduction

    init {
        talon.talon.apply {
            configPeakOutputForward(0.3)
            configPeakOutputReverse(-0.3)
            config_kP(0, 0.85)
            config_kI(0, 0.0006)
            config_kD(0, 0.2)
        }
        encoder.resetPosition(0.degree)
    }

    // position is 0 to 1, where 0 is fully retracted and 1 is fully extended
    fun setPosition(position: Double) {
        val setpoint = maxPos * position
        talon.setPosition(setpoint)
    }

}