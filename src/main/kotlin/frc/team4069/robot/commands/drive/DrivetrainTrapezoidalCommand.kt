package frc.team4069.robot.commands.drive

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand
import frc.team4069.robot.PublishedData
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonConfiguration

class DrivetrainTrapezoidalCommand(val dist: SIUnit<Meter>) : SaturnCommand(Drivetrain) {

    val leftPid = Drivetrain.leftPid
    val rightPid = Drivetrain.rightPid

    val profile = TrapezoidProfile(TrapezoidProfile.Constraints(1.0.feet.velocity.value, 0.5.feet.acceleration.value),
        TrapezoidProfile.State(dist.value, 0.0))

    var start = -1.0

    val datas = mutableListOf<PublishedData>()

    override fun initialize() {
        Drivetrain.leftEncoder.resetPosition(0.meter)
        Drivetrain.rightEncoder.resetPosition(0.meter)
        start = Timer.getFPGATimestamp()
    }

    override fun execute() {
        val t = Timer.getFPGATimestamp() - start
        val setpoint = profile.calculate(t)
        val ff = Drivetrain.feedforward.calculate(setpoint.velocity)
        val leftOutput = ff + leftPid.calculate(Drivetrain.leftVelocity.value, setpoint.velocity)
        val rightOutput = ff + rightPid.calculate(Drivetrain.rightVelocity.value, setpoint.velocity)

        val data = PublishedData(
            true, Timer.getFPGATimestamp(),
            mapOf(
                "Left Reference" to setpoint.velocity,
                "Right Reference" to setpoint.velocity,
                "Left Velocity" to Drivetrain.leftVelocity.value,
                "Right Velocity" to Drivetrain.rightVelocity.value
            )
        )
        datas += data

        Drivetrain.setVoltages(leftOutput.volt, rightOutput.volt)
    }

    override fun isFinished(): Boolean {
        return profile.isFinished(Timer.getFPGATimestamp() - start)
    }

    override fun end(interrupted: Boolean) {
        val json = Json(JsonConfiguration.Stable)
        for (data in datas) {
            Drivetrain.sock?.send(json.stringify(PublishedData.serializer(), data))
        }

        Drivetrain.sock?.send(json.stringify(PublishedData.serializer(), PublishedData(false, 0.0, mapOf())))
    }
}