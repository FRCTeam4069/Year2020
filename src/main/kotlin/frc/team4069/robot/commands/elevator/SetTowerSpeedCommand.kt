package frc.team4069.robot.commands.elevator

import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.saturn.lib.commands.SaturnCommand

class SetTowerSpeedCommand(val speed: Double) : SaturnCommand(TowerOfDoom) {
    override fun initialize() {
        TowerOfDoom.setTowerDutyCycle(speed)
    }

    override fun isFinished(): Boolean {
        return true
    }
}