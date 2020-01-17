package frc.team4069.robot.commands

import edu.wpi.first.wpilibj.Timer
import frc.team4069.robot.subsystem.Flywheel
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.second
import frc.team4069.saturn.lib.types.CSVWritable
import frc.team4069.saturn.lib.util.DeltaTime
import frc.team4069.saturn.lib.util.SaturnNotifier

class FlywheelCharacterizationCommand : SaturnCommand(Flywheel) {
    val data = mutableListOf<List<Double>>() // s, rad/s, rad/s2
    var lastVelocity = 0.0

    val dt = DeltaTime()

    override fun initialize() {
        Flywheel.setDutyCycle(0.8)
    }

    override fun execute() {
        val now = Timer.getFPGATimestamp()
        val delta = dt.updateTime(now.second)
        val velocity = Flywheel.velocity.value

        val acceleration = if (delta != 0.second) {
            (velocity - lastVelocity) / delta.value
        } else {
            0.0
        }

        data.add(listOf(now, velocity, acceleration))
        lastVelocity = velocity
    }

    override fun end(interrupted: Boolean) {
    }
}