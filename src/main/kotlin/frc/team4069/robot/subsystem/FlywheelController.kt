package frc.team4069.robot.subsystem

import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.derived.Velocity
import frc.team4069.saturn.lib.mathematics.units.derived.Volt

class FlywheelController {
    //TODO: Fill these
    val A = 0.0
    val B = SIUnit<Fraction<Velocity<Unitless>, Volt>>(0.0)

    val K = SIUnit<Fraction<Volt, Velocity<Unitless>>>(0.0)

    var x = 0.radian.velocity

    fun update(ref: SIUnit<Velocity<Unitless>>): SIUnit<Volt> {
        val u = K * (ref - x)
        x += A * x + B * u // Update model
        return u
    }
}