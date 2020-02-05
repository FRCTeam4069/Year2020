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

    init {
        println("Init complete")
        talon.inverted = true
//        indexerInput.requestInterrupts {
//            if(it == InterruptableSensorBase.WaitResult.kRisingEdge) {
//                setDutyCycle(0.4)
//            }
//        }
//        indexerInput.setUpSourceEdge(true, false)
//        indexerInput.enableInterrupts()
//
//        elevatorLowest.requestInterrupts {
//            if(it == InterruptableSensorBase.WaitResult.kRisingEdge) {
//                setDutyCycle(0.0)
//            }
//        }
//        elevatorLowest.setUpSourceEdge(true, false)
//        elevatorLowest.enableInterrupts()
    }

    fun setDutyCycle(dutyCycle: Double) {
        talon.set(ControlMode.PercentOutput, dutyCycle)
    }
}

