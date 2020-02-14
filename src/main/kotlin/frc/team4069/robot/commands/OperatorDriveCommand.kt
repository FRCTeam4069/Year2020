package frc.team4069.robot.commands

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand
import kotlin.math.atanh

class OperatorDriveCommand : SaturnCommand(Drivetrain) {

    override fun initialize() {
        Drivetrain.setNeutral()
    }

    override fun execute() {
        val linearPercent = 0.5 * atanh(0.964 * OI.driveSpeed)
//        var angularPercent = OI.driveTurn.pow(3).coerceIn(-1.0..1.0)
        // tanh rescaled to a range of -1.0..1.0 in the given domain
        var angularPercent = 0.5 * atanh(0.964 * OI.driveTurn)
        val sensitivity = when(Drivetrain.gear) {
            Drivetrain.Gear.Low -> Drivetrain.kLowGearSensitivity
            Drivetrain.Gear.High -> Drivetrain.kHighGearSensitivity
        }
        angularPercent *= sensitivity
        Drivetrain.curvatureDrive(linearPercent, angularPercent, linearPercent == 0.0 && angularPercent != 0.0)
    }

}