package frc.team4069.robot

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.robot.subsystem.TowerOfDoom
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.hid.SaturnHID
import frc.team4069.saturn.lib.mathematics.TAU
import frc.team4069.saturn.lib.mathematics.units.Unitless
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitRotationModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.velocity

object Robot : SaturnRobot() {

    private val controls = mutableListOf<SaturnHID<*>>()

    override fun robotInit() {
        +Flywheel
        +TowerOfDoom
        +OI.controller
    }

    override fun teleopInit() {
        Flywheel.enable()
    }

    override fun robotPeriodic() {
        controls.forEach(SaturnHID<*>::update)
    }

    override fun autonomousInit() {
        Flywheel.enable()
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
