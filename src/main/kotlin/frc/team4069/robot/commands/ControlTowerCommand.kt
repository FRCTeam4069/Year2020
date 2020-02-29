package frc.team4069.robot.commands

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.TowerOfDoom
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.commands.SaturnCommand

class ControlTowerCommand : SaturnCommand(TowerOfDoom) {
    override fun execute() {
        val speedScalar = if(Flywheel.enabled) {
            0.8
        } else 0.4
        TowerOfDoom.setTowerDutyCycle(OI.towerSpeed * speedScalar)
    }
}