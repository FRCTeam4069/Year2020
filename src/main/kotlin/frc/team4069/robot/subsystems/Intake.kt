package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object Intake : SaturnSubsystem() {
    private val intakeTalon = TalonSRX(RobotMap.Intake.TALON_ID)

    init {
        intakeTalon.inverted = true
    }

    fun setDutyCycle(demand: Double) {
        intakeTalon.set(ControlMode.PercentOutput, demand)
    }
}