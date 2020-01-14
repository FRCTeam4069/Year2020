package frc.team4069.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.TAU
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity

object Flywheel : SaturnSubsystem() {
    private val talon = TalonSRX(RobotMap.Flywheel.MASTER_TALON_ID)
    private val slave = TalonSRX(RobotMap.Flywheel.SLAVE_TALON_ID)

    private val encoder = Encoder(RobotMap.Flywheel.ENCODER_A, RobotMap.Flywheel.ENCODER_B, true, CounterBase.EncodingType.k1X)

    init {
        slave.follow(talon)
        slave.setInverted(InvertType.InvertMotorOutput)

        encoder.samplesToAverage = 100
        encoder.distancePerPulse = TAU / 2048.0 // encoder ppr = 2048

    }

    fun setDutyCycle(percent: Double) {
        talon.set(ControlMode.PercentOutput, percent)
    }

    val velocity: SIUnit<AngularVelocity>
        get() = encoder.rate.radian.velocity
}