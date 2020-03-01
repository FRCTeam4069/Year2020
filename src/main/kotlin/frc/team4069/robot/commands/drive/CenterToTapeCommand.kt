package frc.team4069.robot.commands.drive

import edu.wpi.first.wpilibj.controller.PIDController
import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Vision
import frc.team4069.saturn.lib.commands.SaturnCommand

class CenterToTapeCommand : SaturnCommand(Drivetrain) {

    val pidController = PIDController(0.6, 0.0, 0.0)
    val linearSpeed = -0.5

    init {
        pidController.apply {
            setpoint = 0.0
        }
    }

    override fun execute() {
        val linearSpeed = OI.driveSpeed
        val output = pidController.calculate(Vision.xOffset.value)
        Drivetrain.tankDrive(linearSpeed - output, linearSpeed + output)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.tankDrive(0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return Vision.targetArea >= 6.7
    }
}