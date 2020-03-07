package frc.team4069.robot.subsystems.flywheel

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration
import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import frc.team4069.robot.PublishedData
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.TAU
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.nt.SaturnNetworkTable
import frc.team4069.saturn.lib.nt.delegate
import frc.team4069.saturn.lib.util.DeltaTime
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
    private val slave = TalonFX(RobotMap.Flywheel.SLAVE_TALON_ID)

    private val encoder =
        Encoder(RobotMap.Flywheel.ENCODER_A, RobotMap.Flywheel.ENCODER_B, true, CounterBase.EncodingType.k1X)

    private lateinit var controllerThread: Notifier

    val controller = FlywheelController()

    val TRENCH_SHOT_PRESET = 3850.rpm
    val CLOSE_SHOT_PRESET = 1700.rpm

    fun enable() {
        controller.enable()
    }

    fun disable() {
        controller.disable()
        setDutyCycle(0.0)
    }

    val enabled: Boolean
        get() = controller.enabled

    fun setReference(speed: SIUnit<AngularVelocity>) {
        controller.reference = speed
    }

    var zmqContext: ZContext? = null
    var sock: ZMQ.Socket? = null

    init {
        talon.inverted = false
        talon.setNeutralMode(NeutralMode.Coast)
        slave.follow(talon)
        slave.setInverted(InvertType.OpposeMaster)
        slave.setNeutralMode(NeutralMode.Coast)
        talon.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 50.0, 0.0, 0.0))
        slave.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 50.0, 0.0, 0.0))

        encoder.setReverseDirection(false)
        encoder.distancePerPulse = TAU / 2048.0 // encoder ppr = 2048
        encoder.samplesToAverage = 25

//        zmqContext = ZContext(2)
//        sock = zmqContext!!.createSocket(SocketType.PUSH)
//        sock!!.bind("tcp://*:5802")

        // For data logging/model tuning
        val json = Json(JsonConfiguration.Stable)
        val delta = DeltaTime()
        val rand = Random()
        val R = 0.018.ohm
        controllerThread = Notifier {
            controller.measuredVelocity = encoderVelocity
            val dt = delta.updateTime(Timer.getFPGATimestamp().second)
            val u = controller.update(dt)
            if (controller.enabled) {
                val now = Timer.getFPGATimestamp()

                // For model tuning
//                val scale = (12.volt - R * talon.supplyCurrent.amp * 2.0) / 12.volt
//                val plantU = u * scale
//                controller.plant.update(controller.plant.x, mat(`1`, `1`).fill(plantU.value), dt.value)

                val data = PublishedData(true, now, mapOf(
                    "Voltage" to u.value,
                    "Velocity" to encoderVelocity.value,
                    "KF Velocity" to velocity.value,
                    "At Goal" to if(controller.atGoal) 1.0 else -1.0
                ))
                sock?.send(json.stringify(PublishedData.serializer(), data))

                talon.set(ControlMode.PercentOutput, u / talon.busVoltage.volt)
            }
        }

        controllerThread.startPeriodic(0.01)
    }

    override fun setNeutral() {
        disable()
        talon.set(ControlMode.PercentOutput, 0.0)
        val json = Json(JsonConfiguration.Stable)
        val data = PublishedData(false, 0.0, mapOf())
        sock?.send(json.stringify(PublishedData.serializer(), data))
    }

    fun setDutyCycle(percent: Double) {
        talon.set(ControlMode.PercentOutput, percent)
    }

    /**
     * The rotational velocity of the shooter as measured by the encoder
     *
     * This value should only be used in the correct step of the KF. Other requirements can be met with the KF estimatated velocity
     */
    val encoderVelocity: SIUnit<AngularVelocity>
        get() = encoder.rate.radian.velocity

    /**
     * The rotational velocity of the shooter as estimated by the KF
     */
    val velocity: SIUnit<AngularVelocity>
        get() = controller.velocity
}
