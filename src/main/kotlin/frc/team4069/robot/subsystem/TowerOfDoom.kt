package frc.team4069.robot.subsystem

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object TowerOfDoom : SaturnSubsystem() {
    private val talon = TalonSRX(3)

    init {
        talon.inverted = true
    }

    fun setDutyCycle(dutyCycle: Double) {
        talon.set(ControlMode.PercentOutput, dutyCycle)
    }
}

