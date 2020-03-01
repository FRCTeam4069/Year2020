package frc.team4069.robot.commands.intake

import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class AdjustIntakePivotCommand(private val desiredPosition: Intake.PivotPosition) : SaturnCommand(Intake) {
    override fun initialize() {
        Intake.setPivotState(desiredPosition)
    }

    override fun isFinished(): Boolean {
        return true
    }
}