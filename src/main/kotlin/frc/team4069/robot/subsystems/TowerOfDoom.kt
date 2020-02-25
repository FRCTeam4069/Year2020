package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.InterruptableSensorBase
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import java.util.concurrent.atomic.AtomicInteger

object TowerOfDoom : SaturnSubsystem() {
    val talon = TalonSRX(RobotMap.Tower.TALON_ID)
    private val elevatorIn = DigitalInput(2)
    private val elevatorOut = DigitalInput(3)

    private var ballCount = AtomicInteger()

    init {
        talon.configFactoryDefault()

        elevatorIn.requestInterrupts {
            if(it == InterruptableSensorBase.WaitResult.kRisingEdge) {
                ballCount.incrementAndGet()
            }
        }
        elevatorIn.enableInterrupts()
        elevatorIn.setUpSourceEdge(true, false)

        elevatorOut.requestInterrupts {
            if(it == InterruptableSensorBase.WaitResult.kRisingEdge) {
                ballCount.decrementAndGet()
            }
        }
        elevatorOut.enableInterrupts()
        elevatorOut.setUpSourceEdge(true, false)
    }

    override fun periodic() {
//        println(ballCount.get())
    }

    fun setTowerDutyCycle(dutyCycle: Double) {
        talon.set(ControlMode.PercentOutput, dutyCycle)
    }
}

