package frc.team4069.robot.commands.elevator

import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.saturn.lib.commands.SaturnCommand

class AutoIndexBallsCommand : SaturnCommand(TowerOfDoom) {
    override fun initialize() {
        TowerOfDoom.setTowerDutyCycle(0.2)
        TowerOfDoom.autoIndexerState = TowerOfDoom.AutoIndexerState.Intaking
    }

    override fun isFinished(): Boolean {
        return TowerOfDoom.autoIndexerState == TowerOfDoom.AutoIndexerState.Pending
    }

    override fun end(interrupted: Boolean) {
        TowerOfDoom.setTowerDutyCycle(0.0)
        TowerOfDoom.autoIndexerState = TowerOfDoom.AutoIndexerState.Off
    }
}