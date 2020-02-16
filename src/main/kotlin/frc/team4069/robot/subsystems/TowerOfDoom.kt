package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.InterruptableSensorBase
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object TowerOfDoom : SaturnSubsystem() {
    val talon = TalonSRX(RobotMap.Tower.TALON_ID)
    val indexer = TalonSRX(RobotMap.Tower.INDEXER_ID)
//    private val indexerInput = DigitalInput(2)
//    private val elevatorLowest = DigitalInput(3)

    private var ballCount = 0.0

    init {
        indexer.inverted = true

        indexer.configContinuousCurrentLimit(5)
        indexer.configPeakCurrentLimit(9)
        indexer.configPeakCurrentDuration(1000)
        indexer.enableCurrentLimit(true)
    }

    fun setTowerDutyCycle(dutyCycle: Double) {
        talon.set(ControlMode.PercentOutput, dutyCycle)
    }

    fun setIndexerDutyCycle(dutyCycle: Double) {
        indexer.set(ControlMode.PercentOutput, dutyCycle)
    }
}

