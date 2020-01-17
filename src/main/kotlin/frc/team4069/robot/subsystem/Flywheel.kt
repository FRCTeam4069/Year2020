package frc.team4069.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import frc.team4069.keigen.get
import frc.team4069.robot.PublishedData
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.TAU
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonConfiguration
import org.zeromq.SocketType
import org.zeromq.ZContext
import org.zeromq.ZMQ

object Flywheel : SaturnSubsystem() {
    private val talon = TalonFX(RobotMap.Flywheel.MASTER_TALON_ID)
    private val encoder =
        Encoder(RobotMap.Flywheel.ENCODER_A, RobotMap.Flywheel.ENCODER_B, true, CounterBase.EncodingType.k1X)

    val controller = FlywheelController()

    // Used to communicate between the DS and robot code for data logging purposes
    // data-logger/logger.py in robot code project is the client
    private var zmqContext: ZContext
    private var sock: ZMQ.Socket

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

        encoder.samplesToAverage = 100
        encoder.distancePerPulse = TAU / 2048.0 // encoder ppr = 2048

        // Set up ZMQ context and bind a PUSH socket on 5802 (one of the team use ports)
        zmqContext = ZContext()
        sock = zmqContext.createSocket(SocketType.PUSH)

        val json = Json(JsonConfiguration.Stable)
        sock.bind("tcp://*:5802")
        GlobalScope.launchFrequency(100.hertz) {
            val u = controller.update()
            if (controller.enabled) {
                val now = Timer.getFPGATimestamp()

                // Logging demand voltage, measured, and estimated states
                val data = PublishedData(
                    true, now, mapOf(
                        "Voltage" to u.value, "Velocity" to velocity.value,
                        "KF Velocity" to controller.observer.xHat[0]
//                        "U Error" to controller.observer.xHat[1]
                    )
                )
                println("Trying to send")
                sock.send(json.stringify(PublishedData.serializer(), data))

                println("Trying to set output voltage")
                talon.set(ControlMode.PercentOutput, u / talon.busVoltage.volt)
            }
        }
    }

    override fun setNeutral() {
        controller.disable()
        talon.set(ControlMode.PercentOutput, 0.0)
        val json = Json(JsonConfiguration.Stable)
        val data = PublishedData(false, 0.0, listOf(), mapOf())

        // enabled = false tells the client to display graphs of the data sent by the server until now
        sock.send(json.stringify(PublishedData.serializer(), data))
    }

    fun setDutyCycle(percent: Double) {
        talon.set(ControlMode.PercentOutput, percent)
    }

    val velocity: SIUnit<AngularVelocity>
        get() = encoder.rate.radian.velocity
}