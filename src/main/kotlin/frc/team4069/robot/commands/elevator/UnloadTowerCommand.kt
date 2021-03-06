package frc.team4069.robot.commands.elevator

import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.saturn.lib.commands.SaturnCommand

class UnloadTowerCommand : SaturnCommand(TowerOfDoom) {
    override fun initialize() {
        TowerOfDoom.setTowerDutyCycle(0.3)
    }

    override fun isFinished(): Boolean {
        return TowerOfDoom.ballCount.get() == 0 && TowerOfDoom.elevatorOut.get()
    }

    override fun end(interrupted: Boolean) {
        TowerOfDoom.setTowerDutyCycle(0.0)
    }
}