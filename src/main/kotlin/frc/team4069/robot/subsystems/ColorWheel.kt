package frc.team4069.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object ColorWheel : SaturnSubsystem() {
    private val spark = CANSparkMax(RobotMap.ColorWheel.SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless)


    override fun setNeutral() {
        //turn off powers to motors
    }

    fun setDutyCycle(demand: Double) {
        spark.set(demand)
    }
}