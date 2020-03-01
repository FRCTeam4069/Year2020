package frc.team4069.robot.commands.flywheel

import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity

class SetFlywheelReferenceCommand(private val reference: SIUnit<AngularVelocity>) : SaturnCommand(Flywheel) {
    override fun initialize() {
        Flywheel.setReference(reference)
    }

    override fun isFinished(): Boolean {
        return true
    }
}