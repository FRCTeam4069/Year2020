package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.InterruptableSensorBase
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem

object TowerOfDoom : SaturnSubsystem() {
    val talon = TalonSRX(RobotMap.Tower.TALON_ID)
//    private val indexerInput = DigitalInput(2)
//    private val elevatorLowest = DigitalInput(3)
    private val ballInput = DigitalInput(8)

    private var ballCount = 0
        get() = field / 2

    init {
        talon.configFactoryDefault()

        ballInput.requestInterrupts {
            if(it == InterruptableSensorBase.WaitResult.kRisingEdge) {
                ballCount += 1
            }
        }
        ballInput.enableInterrupts()
        ballInput.setUpSourceEdge(true, false)
    }

    fun setTowerDutyCycle(dutyCycle: Double) {
        talon.set(ControlMode.PercentOutput, dutyCycle)
    }
}

