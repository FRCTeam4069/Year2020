package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.Num
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.Second
import org.ejml.simple.SimpleMatrix

class UnscentedKalmanFilter<States: Num, Inputs: Num, Outputs: Num>(
    val states: Nat<States>,
    val outputs: Nat<Outputs>,
    val f: (Vector<States>, Vector<Inputs>) -> Vector<States>,
    val h: (Vector<States>, Vector<Inputs>) -> Vector<Outputs>,
    val stateStdDevs: Vector<States>,
    val measurementStdDevs: Vector<Outputs>,
    val useRungeKutta: Boolean = true
) {
    val Q = StateSpaceUtils.makeCovMatrix(states, stateStdDevs)
    val R = StateSpaceUtils.makeCovMatrix(outputs, measurementStdDevs)


    private val pts = MerweScaledSigmaPoints(states, stateStdDevs, 1E-3, 0.0, 2.0)

    var sigmasF = SimpleMatrix(pts.numSigmas, states.num)

    var xHat = zeros(states)

    var P = zeros(states, states)

    init {
        reset()
    }

    fun reset() {
        xHat = zeros(states)
//        P = mat(states, states).fill(1.0, .0, .0, .0, .0, .0, .0, .0, .0)
        P = zeros(states, states)
        sigmasF = SimpleMatrix(pts.numSigmas, states.num)
    }

    fun predict(u: Vector<Inputs>, dt: SIUnit<Second>) {
        val sigmas = pts.sigmaPoints(xHat, P)

        for(i in 0 until pts.numSigmas) {
            val x = sigmas.extractVector(true, i).transpose()

            val fRow = if(useRungeKutta) {
                rungeKutta(f, Vector(x), u, dt).transpose().storage
            } else {
                f(Vector(x), u).transpose().storage
            }
            sigmasF.insertIntoThis(i, 0, fRow)
        }

        val (x, P) = unscentedTransform(states.num, states.num, sigmasF, pts.Wm, pts.Wc, Q.storage)

        xHat = Vector(x.transpose())
        this.P = Matrix(P)
    }
}