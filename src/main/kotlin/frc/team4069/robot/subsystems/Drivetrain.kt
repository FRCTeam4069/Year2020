package frc.team4069.robot.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.OperatorDriveCommand
import frc.team4069.saturn.lib.mathematics.twodim.control.LTVUnicycleTracker
import frc.team4069.saturn.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import frc.team4069.saturn.lib.mathematics.twodim.control.asChassisSpeeds
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.feet
import frc.team4069.saturn.lib.mathematics.units.conversions.meter
import frc.team4069.saturn.lib.motor.SaturnRIOEncoder
import frc.team4069.saturn.lib.motor.rev.SaturnMAX
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem
import kotlin.properties.Delegates

object Drivetrain : TankDriveSubsystem() {

    // Sensitivity of operator inputs for each gear
    const val kHighGearSensitivity = 0.4
    const val kLowGearSensitivity = 0.625

    var gear by Delegates.observable(Gear.High) { _, old, new ->
        // Updates are expensive, should only be done if a new value is set
        if(old != new) {
            when(new) {
                Gear.High -> {
                    shifter.set(DoubleSolenoid.Value.kForward)
                }
                Gear.Low -> {
                    shifter.set(DoubleSolenoid.Value.kReverse)
                }
            }
        }
    }

    private val shifter = DoubleSolenoid(RobotMap.Drivetrain.SHIFTER_FWD, RobotMap.Drivetrain.SHIFTER_BCK)

    override val leftMotor = SaturnMAX(RobotMap.Drivetrain.LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kLeftDrivetrainUnitModel)
    private val leftSlave = SaturnMAX(RobotMap.Drivetrain.LEFT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kLeftDrivetrainUnitModel)
    val leftEncoder = SaturnRIOEncoder(Encoder(RobotMap.Drivetrain.LEFT_ENCODER_A, RobotMap.Drivetrain.LEFT_ENCODER_B), Constants.kLeftDrivetrainUnitModel)

    override val rightMotor = SaturnMAX(RobotMap.Drivetrain.RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kRightDrivetrainUnitModel)
    private val rightSlave = SaturnMAX(RobotMap.Drivetrain.RIGHT_SLAVE, CANSparkMaxLowLevel.MotorType.kBrushless, Constants.kRightDrivetrainUnitModel)
    val rightEncoder = SaturnRIOEncoder(Encoder(RobotMap.Drivetrain.RIGHT_ENCODER_A, RobotMap.Drivetrain.RIGHT_ENCODER_B), Constants.kRightDrivetrainUnitModel)

    override val gyro =  { Rotation2d() }//SaturnPigeon(TowerOfDoom.talon)

    override val driveModel = DifferentialDriveKinematics(2.3563.feet.meter) // wpilib class doesnt have units, so just use for conversions
    override val localization = DifferentialDriveOdometry(gyro(), Pose2d())

    //DO NOT USE
    override val trajectoryTracker = LTVUnicycleTracker(16.409255758939636,
        5.743092173917074,
        5.704580270256358,
        8.841822353363662) { velocity }

    val velocity
        get() = (leftEncoder.velocity + rightEncoder.velocity) / 2.0

    init {
        leftEncoder.resetPosition(0.meter)
        rightEncoder.resetPosition(0.meter)

        leftMotor.outputInverted = false
        rightMotor.outputInverted = true

        leftEncoder.encoder.setReverseDirection(true)

        rightSlave.follow(rightMotor)
        leftSlave.follow(leftMotor)

        leftMotor.canSparkMax.setSmartCurrentLimit(50)
        rightMotor.canSparkMax.setSmartCurrentLimit(50)

        leftMotor.brakeMode = false
        rightMotor.brakeMode = false

        defaultCommand = OperatorDriveCommand()

//        gyro.setFusedHeading(0.0)
    }

    override fun setNeutral() {
        leftMotor.setDutyCycle(0.0)
        rightMotor.setDutyCycle(0.0)
    }

    override fun setOutput(output: TrajectoryTrackerOutput) {
        val wheelSpeeds = driveModel.toWheelSpeeds(output.asChassisSpeeds())

        leftMotor.setVelocity(wheelSpeeds.leftMetersPerSecond.meter.velocity, 0.volt)
        rightMotor.setVelocity(wheelSpeeds.rightMetersPerSecond.meter.velocity, 0.volt)
    }

    enum class Gear {
        Low,
        High
    }
}