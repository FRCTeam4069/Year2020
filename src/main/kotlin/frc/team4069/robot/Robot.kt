package frc.team4069.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.*
import frc.team4069.robot.subsystems.*
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.rpm
import frc.team4069.saturn.lib.mathematics.units.velocity

object Robot : SaturnRobot() {

    private val controls = mutableListOf<SaturnHID<*>>()
    private val pressureSensor = PressureSensor(0)
    private val compressor = Compressor()
    var compressorStarted = false


    override fun robotInit() {
        // Subsystem initializations
        +Drivetrain
        +Flywheel
        +TowerOfDoom
        +Hood
        +Intake
        +Climber
//        Vision

        // Register controllers for control handling
        +OI.controller
        +OI.operatorController
    }

    override fun teleopInit() {
        Flywheel.setReference(2500.rpm)
        Flywheel.enable()
//        Hood.setPosition(1.0)
        ControlIntakeCommand().schedule()
    }

    override fun robotPeriodic() {
        controls.forEach(SaturnHID<*>::update)

        if(pressureSensor.pressure < 100.0 && !compressorStarted) {
            compressor.start()
            compressorStarted = true
        } else if(pressureSensor.pressure >= 110.0) {
            compressor.stop()
            compressorStarted = false
        }

        println(Hood.talon.selectedSensorPosition)
    }

    override fun autonomousInit() {

//        SetDrivetrainSpeedCommand(6.feet.velocity).schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    operator fun SaturnHID<*>.unaryPlus() {
        controls += this
    }
}

fun main() {
    Robot.start()
}
