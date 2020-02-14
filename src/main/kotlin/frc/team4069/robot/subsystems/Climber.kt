package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object Climber : SaturnSubsystem() {

    private val motor = CANSparkMax(RobotMap.Climber.SPARK1_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val slave = CANSparkMax(RobotMap.Climber.SPARK2_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val slideyBoye = TalonSRX(32)

    init {
        slave.follow(motor, true)
        motor.idleMode = CANSparkMax.IdleMode.kBrake
        slave.idleMode = CANSparkMax.IdleMode.kBrake
    }

    fun setDutyCycle(demand: Double) {
        motor.set(demand)
    }

    fun setSlide(demand: Double) {
        slideyBoye.set(ControlMode.PercentOutput, demand)
    }
}
