package frc.team4069.robot.commands

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Hood
import frc.team4069.saturn.lib.commands.SaturnCommand

class ControlHoodCommand : SaturnCommand(Hood) {
    override fun execute() {
        Hood.setDutyCycle(OI.hoodSpeed)
    }
}