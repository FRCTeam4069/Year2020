package frc.team4069.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.shuffleboard.logging.tab
import kotlin.random.Random

object Robot : SaturnRobot() {

    val talon = TalonSRX(4)

    override fun robotInit() {
        talon.configSelectedFeedbackSensor(FeedbackDevice.SoftwareEmulatedSensor)
        tab("Robot") {
            list("screm") {
                textView("talon pos", { talon.selectedSensorPosition }) {

                }
            }
        }
    }

    override fun teleopInit() {
        talon.set(ControlMode.PercentOutput, 0.5)
    }
}

fun main() {
    Robot.start()
}
