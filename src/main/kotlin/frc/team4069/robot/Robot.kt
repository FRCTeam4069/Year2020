package frc.team4069.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.TestCommand
import frc.team4069.robot.commands.auto.EnemyTrenchAuto
import frc.team4069.robot.commands.auto.FriendlyTrenchAuto
import frc.team4069.robot.commands.climber.ControlClimberCommand
import frc.team4069.robot.commands.colourwheel.ControlColourWheelCommand
import frc.team4069.robot.commands.intake.ControlIntakeCommand
import frc.team4069.robot.commands.drive.OperatorDriveCommand
import frc.team4069.robot.commands.drive.DrivetrainTests
import frc.team4069.robot.commands.elevator.ControlTowerCommand
import frc.team4069.robot.subsystems.*
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.robot.util.InterpolatableDouble
import frc.team4069.robot.util.PressureSensor
import frc.team4069.robot.util.extrapolate
import frc.team4069.robot.util.interpolatableMapOf
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.shuffleboard.logging.sendableChooser

object Robot : SaturnRobot() {

    private val controls = mutableListOf<SaturnHID<*>>()
    private val pressureSensor = PressureSensor(0)
    private val compressor = Compressor()
    var compressorStarted = false

    private val autoChooser = sendableChooser(
        "Opposite Trench Auto" to EnemyTrenchAuto(),
        "Same Side Trench Auto" to FriendlyTrenchAuto()
    )


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
        Trajectories

        SmartDashboard.putData("Autonomous Mode", autoChooser)

        // Register controllers for control handling
        +OI.controller
        +OI.operatorController

        val map = interpolatableMapOf(
            1.0 to InterpolatableDouble(3.0),
            2.0 to InterpolatableDouble(5.0)
        )
        println("EXTRAP ${map.extrapolate(0.0)}")
    }

    override fun teleopInit() {
        OperatorDriveCommand().schedule()
        ControlClimberCommand().schedule()
        ControlIntakeCommand().schedule()
        ControlColourWheelCommand().schedule()
        ControlTowerCommand().schedule()
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
        autoChooser.selected.schedule()
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
