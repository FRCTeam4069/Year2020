package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object Climber : SaturnSubsystem() {

    private val motor = CANSparkMax(RobotMap.Climber.SPARK1_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.encoder
    private val slave = CANSparkMax(RobotMap.Climber.SPARK2_ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val brakeSolenoid = DoubleSolenoid(RobotMap.Climber.BRAKE_SOLENOID_FWD, RobotMap.Climber.BRAKE_SOLENOID_BACK)
    private var brakeEngaged = false

    val sliderTalon = TalonSRX(RobotMap.Climber.SLIDE_TALON)

    init {
        encoder.position = 0.0
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0F)
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -73F)
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true)
        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true)
        slave.follow(motor, true)
        motor.idleMode = CANSparkMax.IdleMode.kBrake
        slave.idleMode = CANSparkMax.IdleMode.kBrake
    }

    fun setDutyCycle(demand: Double) {
        if(!brakeEngaged) {
            motor.set(demand)
        }
    }

    fun engageBrake() {
        brakeEngaged = true
        brakeSolenoid.set(DoubleSolenoid.Value.kForward)
        motor.set(0.0)
    }

    fun disengageBrake() {
        brakeEngaged = false
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse)
    }

    fun setSlide(demand: Double) {
        sliderTalon.set(ControlMode.PercentOutput, demand)
    }
}
