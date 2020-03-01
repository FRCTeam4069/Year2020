package frc.team4069.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team4069.robot.commands.auto.FriendlyTrenchAuto
import frc.team4069.robot.commands.climber.ControlClimberCommand
import frc.team4069.robot.commands.colourwheel.ControlColourWheelCommand
import frc.team4069.robot.commands.intake.ControlIntakeCommand
import frc.team4069.robot.commands.drive.OperatorDriveCommand
import frc.team4069.robot.commands.drive.DrivetrainTests
import frc.team4069.robot.subsystems.*
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.rpm
import frc.team4069.saturn.lib.subsystem.TrajectoryTrackerCommand

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
        +ColorWheel
        +Intake
        +Climber
        Vision

        // Register controllers for control handling
//        +OI.controller
        +OI.operatorController
    }

    override fun teleopInit() {
//        Flywheel.enable()
        OperatorDriveCommand().schedule()
//        ControlHoodCommand().schedule()
        ControlClimberCommand().schedule()
        ControlIntakeCommand().schedule()
        ControlColourWheelCommand().schedule()
//        Hood.setPosition(0.75)
    }

    override fun robotPeriodic() {
        controls.forEach(SaturnHID<*>::update)

        if (pressureSensor.pressure < 100.0 && !compressorStarted) {
            compressor.start()
            compressorStarted = true
        } else if (pressureSensor.pressure >= 110.0) {
            compressor.stop()
            compressorStarted = false
        }
    }

    override fun autonomousInit() {
//        Flywheel.enable()
//        Flywheel.setReference(1000.rpm)
        FriendlyTrenchAuto().schedule()
    }

    override fun autonomousPeriodic() {
    }

    override fun disabledInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    override fun testInit() {
        CommandScheduler.getInstance().enable()
        TestCommand(Drivetrain).withPhases(
            DrivetrainTests.VerifyEncodersForward(),
            DrivetrainTests.VerifyEncodersBackwards(),
            DrivetrainTests.VerifyEncodersTurning()
        ).schedule()
    }

    operator fun SaturnHID<*>.unaryPlus() {
        controls += this
    }
}

fun main() {
    Robot.start()
}
