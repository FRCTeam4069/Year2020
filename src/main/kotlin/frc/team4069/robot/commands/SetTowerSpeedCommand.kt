package frc.team4069.robot.commands

import frc.team4069.robot.subsystem.TowerOfDoom
import frc.team4069.saturn.lib.commands.SaturnCommand

class SetTowerSpeedCommand(val dutyCycle: Double) : SaturnCommand(TowerOfDoom) {
    override fun initialize() {
        TowerOfDoom.setDutyCycle(dutyCycle)
    }

    override fun isFinished(): Boolean {
        return true
    }
}