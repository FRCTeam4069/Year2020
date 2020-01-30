package frc.team4069.robot.commands

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.drivetrain.Drivetrain
import frc.team4069.saturn.lib.commands.SaturnCommand
import kotlin.math.pow

class OperatorDriveCommand : SaturnCommand(Drivetrain) {

    override fun initialize() {
        Drivetrain.setNeutral()
    }

    override fun execute() {
        val linearPercent = OI.driveSpeed
        var angularPercent = OI.driveTurn.pow(3).coerceIn(-1.0..1.0)
        val sensitivity = when(Drivetrain.gear) {
            Drivetrain.Gear.Low -> Drivetrain.kLowGearSensitivity
            Drivetrain.Gear.High -> Drivetrain.kHighGearSensitivity
        }
        angularPercent *= sensitivity
        Drivetrain.curvatureDrive(linearPercent, angularPercent, linearPercent == 0.0 && angularPercent != 0.0)
    }

}