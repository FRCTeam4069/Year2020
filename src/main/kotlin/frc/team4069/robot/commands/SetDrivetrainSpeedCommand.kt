package frc.team4069.robot.commands

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import frc.team4069.robot.Constants
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.conversions.feetPerSecond

class SetDrivetrainSpeedCommand(val spd: SIUnit<LinearVelocity>) : SaturnCommand(Drivetrain) {
    private val leftPid = PIDController(0.1, 0.0, 0.0)
    private val rightPid = PIDController(0.1, 0.0, 0.0)

    private val feedforward = SimpleMotorFeedforward(Constants.DRIVETRAIN_KS.value, Constants.DRIVETRAIN_KV.value, Constants.DRIVETRAIN_KA.value)

    override fun initialize() {
        leftPid.reset()
        rightPid.reset()
    }

    override fun execute() {
        val leftOutput = feedforward.calculate(spd.value)
        val rightOutput = feedforward.calculate(spd.value)

        val Vbus = RobotController.getBatteryVoltage()
        Drivetrain.tankDrive(leftOutput / Vbus, rightOutput / Vbus)
        println("DESIRED: ${spd.feetPerSecond}, ACT: ${Drivetrain.velocity.feetPerSecond}")
    }
}