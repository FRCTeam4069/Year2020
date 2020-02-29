package frc.team4069.robot.commands

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Climber
import frc.team4069.saturn.lib.commands.SaturnCommand

class ControlClimberCommand : SaturnCommand(Climber) {

    override fun execute() {
        Climber.setDutyCycle(OI.climberSpeed)
        Climber.setSlide(OI.sliderSpeed)
    }
}