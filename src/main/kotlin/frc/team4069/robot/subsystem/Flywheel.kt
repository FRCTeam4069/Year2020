package frc.team4069.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.MotorCommutation
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import frc.team4069.keigen.*
import frc.team4069.robot.PublishedData
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.TAU
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.GlobalScope
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonConfiguration
import org.zeromq.SocketType
import org.zeromq.ZContext
import org.zeromq.ZMQ
import java.util.*

object Flywheel : SaturnSubsystem() {
    private val talon = TalonFX(RobotMap.Flywheel.MASTER_TALON_ID)
    private val encoder =
        Encoder(RobotMap.Flywheel.ENCODER_A, RobotMap.Flywheel.ENCODER_B, true)

    val controller = FlywheelController()

    fun enable() {
        controller.enable()
    }

    fun disable() {
        controller.disable()
    }

    fun setReference(speed: SIUnit<AngularVelocity>) {
        controller.reference = speed
    }

    init {
        talon.inverted = true
        talon.setNeutralMode(NeutralMode.Coast)

        encoder.distancePerPulse = TAU / 2048.0 // encoder ppr = 2048

        GlobalScope.launchFrequency(100.hertz) {
            controller.measuredVelocity = encoderVelocity
            val u = controller.update()
            if (controller.enabled) {
                val now = Timer.getFPGATimestamp()

                talon.set(ControlMode.PercentOutput, u / talon.busVoltage.volt)
            }
        }
    }

    override fun setNeutral() {
        controller.disable()
        talon.set(ControlMode.PercentOutput, 0.0)
    }

    fun setDutyCycle(percent: Double) {
        talon.set(ControlMode.PercentOutput, percent)
    }

    /**
     * The rotational velocity of the shooter as measured by the encoder
     *
     * This value should only be used in the correct step of the KF. Other requirements can be met with the KF estimatated velocity
     */
    private val encoderVelocity: SIUnit<AngularVelocity>
        get() = encoder.rate.radian.velocity

    /**
     * The rotational velocity of the shooter as estimated by the KF
     */
    val velocity: SIUnit<AngularVelocity>
        get() = controller.observer.xHat[0].radian.velocity
}