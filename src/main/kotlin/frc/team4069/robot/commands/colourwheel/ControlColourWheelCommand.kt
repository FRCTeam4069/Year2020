package frc.team4069.robot.commands.colourwheel

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.ColorWheel
import frc.team4069.saturn.lib.commands.SaturnCommand

class ControlColourWheelCommand : SaturnCommand(ColorWheel) {
    override fun execute() {
        ColorWheel.setDutyCycle(OI.colourWheelSpeed)
    }
}