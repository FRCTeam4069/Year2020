package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.conversions.degree
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitRotationModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import frc.team4069.saturn.lib.shuffleboard.logging.tab

object Hood : SaturnSubsystem() {

    private val talon = TalonSRX(RobotMap.Hood.TALON_ID)
    private val RAW_MAX = 30150.STU

    init {
        talon.apply {
            inverted = true
            config_kP(0, 0.5)
            configForwardSoftLimitThreshold(RAW_MAX.value.toInt())
            configForwardSoftLimitEnable(true)
            configReverseSoftLimitThreshold(0)
            configReverseSoftLimitEnable(true)

            selectedSensorPosition = 0
        }
    }

    override fun periodic() {
        println(talon.closedLoopError)
    }

    fun setDutyCycle(demand: Double) {
        talon.set(ControlMode.PercentOutput, demand)
    }

    // position is 0 to 1, where 0 is fully retracted and 1 is fully extended
    fun setPosition(position: Double) {
        val setpoint = RAW_MAX * position
        talon.set(ControlMode.Position, setpoint.value)
    }

}