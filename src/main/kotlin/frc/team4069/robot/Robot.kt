package frc.team4069.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.commands.ControlIntakeCommand
import frc.team4069.robot.commands.OperatorDriveCommand
import frc.team4069.robot.commands.drive.DrivetrainTests
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.mathematics.twodim.geometry.xU
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.subsystem.TrajectoryTrackerCommand
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonConfiguration

object Robot : SaturnRobot() {

    private val controls = mutableListOf<SaturnHID<*>>()
    private val pressureSensor = PressureSensor(0)
    private val compressor = Compressor()
    var compressorStarted = false


    override fun robotInit() {
        // Subsystem initializations
        +Drivetrain
//        +Flywheel
//        +TowerOfDoom
//        +Hood
//        +Intake
//        +Climber
//        Vision

        // Register controllers for control handling
//        +OI.controller
//        +OI.operatorController
    }

    override fun teleopInit() {
//        Flywheel.enable()
//        OperatorDriveCommand().schedule()
//        ControlHoodCommand().schedule()
//        ControlClimberCommand().schedule()
//        ControlIntakeCommand().schedule()
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
        val json = Json(JsonConfiguration.Stable)
        val datas = mutableListOf<PublishedData>()
        TrajectoryTrackerCommand(Drivetrain,
            Constants.RAMSETE_B,
            Constants.RAMSETE_ZETA,
            trajectory = Trajectories.testTrajectory,
            leftPid = Drivetrain.leftPid,
            rightPid = Drivetrain.rightPid,
            feedforward = Drivetrain.feedforward,
            resetPose = true,
            velocityConsumer = { left, right ->
                val data = PublishedData(
                    true, Timer.getFPGATimestamp(),
                    mapOf(
                        "Left Reference" to left.value,
                        "Right Reference" to right.value,
                        "Left Velocity" to Drivetrain.leftVelocity().value,
                        "Right Velocity" to Drivetrain.rightVelocity().value
                    )
                )
                datas += data
            }).andThen({
            for (data in datas) {
                Drivetrain.sock?.send(json.stringify(PublishedData.serializer(), data))
            }
            Drivetrain.sock?.send(json.stringify(PublishedData.serializer(), PublishedData(false, 0.0, mapOf())))
        }, arrayOf())
            .schedule()
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
