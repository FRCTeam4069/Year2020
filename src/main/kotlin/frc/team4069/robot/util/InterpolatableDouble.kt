package frc.team4069.robot.util

import frc.team4069.saturn.lib.types.Interpolatable

inline class InterpolatableDouble(val value: Double) : Interpolatable<InterpolatableDouble>, Extrapolatable<InterpolatableDouble> {

    private fun func(endValue: InterpolatableDouble, t: Double): InterpolatableDouble {
        val dydx = endValue - this
        return dydx * t + this
    }

    override fun interpolate(endValue: InterpolatableDouble, t: Double): InterpolatableDouble {
        return when {
            t <= 0.0 -> this
            t >= 1.0 -> endValue
            else -> {
                func(endValue, t)
            }
        }
    }

    override fun extrapolate(endValue: InterpolatableDouble, t: Double): InterpolatableDouble {
        return func(endValue, t)
    }

    operator fun plus(other: InterpolatableDouble) = InterpolatableDouble(value + other.value)
    operator fun plus(other: Double) = InterpolatableDouble(value + other)
    operator fun minus(other: InterpolatableDouble) = InterpolatableDouble(value - other.value)
    operator fun minus(other: Double) = InterpolatableDouble(value - other)
    operator fun times(other: InterpolatableDouble) = InterpolatableDouble(value * other.value)
    operator fun times(other: Double) = InterpolatableDouble(value * other)
    operator fun div(other: InterpolatableDouble) = InterpolatableDouble(value / other.value)
    operator fun div(other: Double) = InterpolatableDouble(value / other)
}