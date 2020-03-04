package frc.team4069.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.InterruptableSensorBase
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.elevator.ControlTowerCommand
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import java.util.concurrent.atomic.AtomicInteger

object TowerOfDoom : SaturnSubsystem() {
    private val spark = CANSparkMax(RobotMap.Tower.SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val elevatorIn = DigitalInput(2)
    val elevatorOut = DigitalInput(3)

    var ballCount = AtomicInteger(0)

    var autoIndexerState = AutoIndexerState.Off
    private var backdriveUntilTopRising = false

    init {
        elevatorIn.requestInterrupts {
            when (it) {
                InterruptableSensorBase.WaitResult.kRisingEdge -> {
                    if (autoIndexerState != AutoIndexerState.Backdriving) {
                        ballCount.incrementAndGet()
                    }else {
                        if(!backdriveUntilTopRising) {
                            autoIndexerState = AutoIndexerState.Pending
                        }
                    }
                }
                InterruptableSensorBase.WaitResult.kFallingEdge -> {
                    if (ballCount.get() >= 2 && autoIndexerState == AutoIndexerState.Intaking) {
                        autoIndexerState = AutoIndexerState.Backdriving
                        backdriveUntilTopRising = !elevatorOut.get()
                        setTowerDutyCycle(-0.1)
                    }
                }
            }
        }
        elevatorIn.enableInterrupts()
        elevatorIn.setUpSourceEdge(true, true)

        elevatorOut.requestInterrupts {
            when (it) {
                InterruptableSensorBase.WaitResult.kFallingEdge -> {
                    if (Flywheel.enabled) {
                        if(ballCount.decrementAndGet() < 0) {
                            ballCount.set(0)
                        }
                    }
                }
                InterruptableSensorBase.WaitResult.kRisingEdge -> {
                    if(autoIndexerState == AutoIndexerState.Backdriving && backdriveUntilTopRising) {
                        backdriveUntilTopRising = false
                        autoIndexerState = AutoIndexerState.Pending
                    }
                }
            }
        }
        elevatorOut.enableInterrupts()
        elevatorOut.setUpSourceEdge(true, true)

//        defaultCommand = ControlTowerCommand()
    }

    fun setTowerDutyCycle(dutyCycle: Double) {
        spark.set(dutyCycle)
    }

    enum class AutoIndexerState {
        Off,
        Intaking,
        Backdriving,
        Pending
    }
}

