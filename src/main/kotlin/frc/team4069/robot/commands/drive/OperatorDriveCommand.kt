package frc.team4069.robot.commands.drive

import edu.wpi.first.wpilibj.SlewRateLimiter
import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand
import kotlin.math.atanh
import kotlin.math.pow
import kotlin.math.sign

class OperatorDriveCommand : SaturnCommand(Drivetrain) {

    private val linearSlewRateLimiter = SlewRateLimiter(1.4)
    private val angularSlewRateLimiter = SlewRateLimiter(5.0)

    override fun initialize() {
        Drivetrain.setNeutral()
    }

    override fun execute() {
//        val linearPercent = 0.5 * atanh(0.964 * linearSlewRateLimiter.calculate(OI.driveSpeed))
        val limitedLinear = linearSlewRateLimiter.calculate(OI.driveSpeed)
        val linearPercent = sign(limitedLinear) * limitedLinear.pow(2)
//        var angularPercent = OI.driveTurn.pow(3).coerceIn(-1.0..1.0)
        // tanh rescaled to a range of -1.0..1.0 in the given domain
        var angularPercent = angularSlewRateLimiter.calculate(OI.driveTurn).pow(3)
        val sensitivity = when(Drivetrain.gear) {
            Drivetrain.Gear.Low -> Drivetrain.kLowGearSensitivity
            Drivetrain.Gear.High -> Drivetrain.kHighGearSensitivity
        }
        angularPercent *= if(linearPercent == 0.0) {
            sensitivity
        } else Drivetrain.kHighGearMovingSensitivity

        Drivetrain.curvatureDrive(linearPercent, angularPercent, linearPercent < 0.2 && angularPercent != 0.0)
    }

}