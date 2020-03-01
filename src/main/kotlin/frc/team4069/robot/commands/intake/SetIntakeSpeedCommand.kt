package frc.team4069.robot.commands.intake

import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class SetIntakeSpeedCommand(private val speed: Double) : SaturnCommand(Intake) {
    override fun initialize() {
        Intake.setDutyCycle(speed)
    }

    override fun isFinished(): Boolean {
        return true
    }
}