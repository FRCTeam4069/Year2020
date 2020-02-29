package frc.team4069.robot.commands

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Intake
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.saturn.lib.commands.SaturnCommand

class ControlIntakeCommand : SaturnCommand(Intake) {

    override fun execute() {
        val spd = OI.intakeSpeed
        Intake.setDutyCycle(spd)
    }
}
