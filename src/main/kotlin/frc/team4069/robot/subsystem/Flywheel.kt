package frc.team4069.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.github.doyaaaaaken.kotlincsv.dsl.csvWriter
import edu.wpi.first.wpilibj.CounterBase
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Timer
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.TAU
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.GlobalScope

object Flywheel : SaturnSubsystem() {
    private val talon = TalonSRX(RobotMap.Flywheel.MASTER_TALON_ID)
    private val slave = TalonSRX(RobotMap.Flywheel.SLAVE_TALON_ID)

    private val encoder = Encoder(RobotMap.Flywheel.ENCODER_A, RobotMap.Flywheel.ENCODER_B, true, CounterBase.EncodingType.k1X)

    val controller = FlywheelController()
    private val enabledVoltages = mutableListOf(listOf<Any>())
    private var enabledVelocities = mutableListOf(listOf<Any>())

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
        slave.follow(talon)

        encoder.samplesToAverage = 100
        encoder.distancePerPulse = TAU / 2048.0 // encoder ppr = 2048
        enabledVoltages.add(listOf("Time", "Voltage"))
        enabledVelocities.add(listOf("Time", "Velocity"))

        GlobalScope.launchFrequency(100.hertz) {
            val u = controller.update()
            if(controller.enabled) {
                val now = Timer.getFPGATimestamp()
                enabledVoltages.add(listOf(now, u.value))
                enabledVelocities.add(listOf(now, velocity.value))
                talon.set(ControlMode.PercentOutput, u / 12.volt)
            }
        }
    }

    override fun setNeutral() {
        controller.disable()
        talon.set(ControlMode.PercentOutput, 0.0)
        csvWriter().open("/home/lvuser/ShooterVoltage.csv", append = false) {
            writeAll(enabledVoltages)
        }
        enabledVoltages.clear()
        enabledVoltages.add(listOf("Time", "Voltage"))

        csvWriter().open("/home/lvuser/ShooterVelocity.csv", append = false) {
            writeAll(enabledVelocities)
        }
        enabledVelocities.clear()
        enabledVelocities.add(listOf("Time", "Velocity"))
    }

    fun setDutyCycle(percent: Double) {
        talon.set(ControlMode.PercentOutput, percent)
    }

    val velocity: SIUnit<AngularVelocity>
        get() = encoder.rate.radian.velocity
}