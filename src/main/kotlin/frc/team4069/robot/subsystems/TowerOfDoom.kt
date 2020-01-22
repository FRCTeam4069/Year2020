package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object TowerOfDoom : SaturnSubsystem() {
    private val talon = TalonSRX(RobotMap.Tower.TALON_ID)

    init {
        talon.inverted = true
    }

    fun setDutyCycle(dutyCycle: Double) {
        talon.set(ControlMode.PercentOutput, dutyCycle)
    }
}

