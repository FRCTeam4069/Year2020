package frc.team4069.robot.commands

import edu.wpi.first.wpilibj.controller.PIDController
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Vision
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.conversions.degree

class CenterToTapeCommand : SaturnCommand(Drivetrain) {

    val pidController = PIDController(0.5, 0.0, 0.1)

    init {
        pidController.apply {
            setpoint = 0.0
            setTolerance(0.02, 1.2)
        }
    }

    override fun execute() {
        val output = pidController.calculate(Vision.xOffset.value)
        Drivetrain.tankDrive(-output, output)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.tankDrive(0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return pidController.atSetpoint()
    }
}