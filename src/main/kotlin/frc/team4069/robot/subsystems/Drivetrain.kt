package frc.team4069.robot.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.drive.OperatorDriveCommand
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.degree
import frc.team4069.saturn.lib.motor.SaturnRIOEncoder
import frc.team4069.saturn.lib.motor.rev.SaturnMAX
import frc.team4069.saturn.lib.sensors.SaturnPigeon
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem
import org.zeromq.ZContext
import org.zeromq.ZMQ
import kotlin.properties.Delegates

object Drivetrain : TankDriveSubsystem() {

    // Sensitivity of operator inputs for each gear
    const val kHighGearSensitivity = 0.35
    const val kHighGearMovingSensitivity = 0.45
    const val kLowGearSensitivity = 0.625

    var gear by Delegates.observable(Gear.High) { _, old, new ->
        // Updates are expensive, should only be done if a new value is set
        if (old != new) {
            when (new) {
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

    override val leftMotor = SaturnMAX(
        RobotMap.Drivetrain.LEFT_MASTER,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.kLeftDrivetrainUnitModel
    )
    private val leftSlave = SaturnMAX(
        RobotMap.Drivetrain.LEFT_SLAVE,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.kLeftDrivetrainUnitModel
    )
    val leftEncoder = SaturnRIOEncoder(
        Encoder(RobotMap.Drivetrain.LEFT_ENCODER_A, RobotMap.Drivetrain.LEFT_ENCODER_B, true, CounterBase.EncodingType.k1X),
        Constants.kLeftDrivetrainUnitModel
    )
    val leftPid = PIDController(0.6, 0.0, 0.0)

    override val rightMotor = SaturnMAX(
        RobotMap.Drivetrain.RIGHT_MASTER,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.kRightDrivetrainUnitModel
    )
    private val rightSlave = SaturnMAX(
        RobotMap.Drivetrain.RIGHT_SLAVE,
        CANSparkMaxLowLevel.MotorType.kBrushless,
        Constants.kRightDrivetrainUnitModel
    )
    val rightEncoder = SaturnRIOEncoder(
        Encoder(RobotMap.Drivetrain.RIGHT_ENCODER_A, RobotMap.Drivetrain.RIGHT_ENCODER_B, false, CounterBase.EncodingType.k1X),
        Constants.kRightDrivetrainUnitModel
    )
    val rightPid = PIDController(0.6, 0.0, 0.0)

    val feedforward = SimpleMotorFeedforward(
        Constants.DRIVETRAIN_KS.value, Constants.DRIVETRAIN_KV.value,
        Constants.DRIVETRAIN_KA.value
    )

    private val _gyro = SaturnPigeon(Climber.sliderTalon)
    override val gyro = { _gyro() }

    override val kinematics = DifferentialDriveKinematics(1.2) // 0.5717 tuned
    override val localization = DifferentialDriveOdometry(gyro(), Pose2d())

    val velocity
        get() = (leftEncoder.velocity + rightEncoder.velocity) / 2.0

    val distance
        get() = (leftEncoder.position + rightEncoder.position) / 2.0

    override val leftVelocity get() = leftEncoder.velocity
    override val rightVelocity get() = rightEncoder.velocity

    private var zmqContext: ZContext? = null
    var sock: ZMQ.Socket? = null

    init {
        leftEncoder.encoder.samplesToAverage = 10
        rightEncoder.encoder.samplesToAverage = 10

        leftMotor.outputInverted = false
        rightMotor.outputInverted = true

        rightSlave.follow(rightMotor)
        leftSlave.follow(leftMotor)

        leftMotor.canSparkMax.setSmartCurrentLimit(50)
        rightMotor.canSparkMax.setSmartCurrentLimit(50)
        leftSlave.canSparkMax.setSmartCurrentLimit(50)
        rightSlave.canSparkMax.setSmartCurrentLimit(50)

        leftMotor.brakeMode = true
        rightMotor.brakeMode = true
        leftSlave.brakeMode = true
        rightSlave.brakeMode = true

        defaultCommand = OperatorDriveCommand()
        _gyro.setFusedHeading(0.0)

    }

    override fun lateInit() {
        localization.resetPosition(Pose2d(), gyro())
        Notifier {
            localization.update(gyro(), leftEncoder.position.value, rightEncoder.position.value)
        }.startPeriodic(1.0 / 100.0)
    }

    override fun setNeutral() {
        leftMotor.setDutyCycle(0.0)
        rightMotor.setDutyCycle(0.0)
    }

    override fun autoReset() {
        leftEncoder.resetPosition(0.meter)
        rightEncoder.resetPosition(0.meter)
        _gyro.setFusedHeading(0.0)
    }

    fun resetGyro(angle: SIUnit<Unitless>) {
        _gyro.setFusedHeading(angle.degree)
    }

    fun arcadeDrive(linear: Double, angular: Double) {
        leftMotor.setDutyCycle(linear + angular)
        rightMotor.setDutyCycle(linear - angular)
    }

    enum class Gear {
        Low,
        High
    }
}