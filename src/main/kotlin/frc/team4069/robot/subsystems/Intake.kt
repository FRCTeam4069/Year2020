package frc.team4069.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ControlType
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object Intake : SaturnSubsystem() {
    private val intakeSpark = CANSparkMax(RobotMap.Intake.SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val pivotSpark = CANSparkMax(RobotMap.Intake.PIVOT_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val pivotEncoder = pivotSpark.encoder

    init {
        intakeSpark.inverted = false
        pivotEncoder.position = 0.0

        pivotSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0F)
        pivotSpark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 27F)
        pivotSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true)
        pivotSpark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true)
    }

    fun setDutyCycle(demand: Double) {
        intakeSpark.set(demand)
    }

    fun setPivotState(pos: PivotPosition) {
        //TODO: PID burns motor, maybe dont.
        when(pos) {
            PivotPosition.Retracted -> {
                pivotSpark.set(-0.4)
            }
            PivotPosition.Extended -> {
                pivotSpark.set(0.4)
            }
        }
    }

    enum class PivotPosition {
        Extended,
        Retracted
    }
}