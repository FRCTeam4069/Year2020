package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpiutil.math.Num
import frc.team4069.saturn.lib.mathematics.matrix.Vector
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.Second

private operator fun <N: Num> Double.times(vec: Vector<N>) = vec.times(this)

/**
 * Performs 4th order Runge-Kutta integration of dx/dt = f(x, u) for dt.
 *
 * @param f  The function to integrate. It must take two arguments x and u.
 * @param x  The initial value of x.
 * @param u  The value u held constant over the integration period
 * @param dt The time over which to integrate
 */
fun <States: Num, Inputs: Num> rungeKutta(f: (Vector<States>, Vector<Inputs>) -> Vector<States>, x: Vector<States>, u: Vector<Inputs>, dt: SIUnit<Second>): Vector<States> {
    val halfDt = dt * 0.5

    val k1 = f(x, u)
    val k2 = f(x + k1 * halfDt.value, u)
    val k3 = f(x + k2 * halfDt.value, u)
    val k4 = f(x + k3 * dt.value, u)

    return x + dt.value / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
}
