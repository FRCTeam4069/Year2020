package frc.team4069.robot.util

interface Extrapolatable<V> {
    fun extrapolate(endValue: V, t: Double): V
}