package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpilibj.math.Drake
import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.Num
import frc.team4069.saturn.lib.mathematics.matrix.Vector
import frc.team4069.saturn.lib.mathematics.matrix.eye
import frc.team4069.saturn.lib.mathematics.matrix.zeros
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.Second

typealias StateFunction<States, Inputs> = (Vector<States>, Vector<Inputs>) -> Vector<States>
typealias OutputFunction<States, Inputs, Outputs> = (Vector<States>, Vector<Inputs>) -> Vector<Outputs>

class ExtendedKalmanFilter<States: Num, Inputs: Num, Outputs: Num>(
    val states: Nat<States>, val inputs: Nat<Inputs>, val outputs: Nat<Outputs>,
    val f: StateFunction<States, Inputs>,
    val h: OutputFunction<States, Inputs, Outputs>,
    stateStdDevs: Vector<States>,
    measurementStdDevs: Vector<Outputs>,
    dt: SIUnit<Second>,
    val useRungeKutta: Boolean = true
) {

    var xHat: Vector<States> = zeros(states)
    private var P: Matrix<States, States>
    private val contQ: Matrix<States, States> = StateSpaceUtils.makeCovMatrix(states, stateStdDevs)
    private val contR: Matrix<Outputs, Outputs> = StateSpaceUtils.makeCovMatrix(outputs, measurementStdDevs)
    private var discR: Matrix<Outputs, Outputs>

    private val initP: Matrix<States, States>

    init {
        reset()

        val contA = numericalJacobianX(states, states, f, xHat, zeros(inputs))
        val C = numericalJacobianX(outputs, states, h, xHat, zeros(inputs))

        val discPair = StateSpaceUtils.discretizeAQTaylor(contA, contQ, dt.value)
        val discA = discPair.first
        val discQ = discPair.second

        discR = StateSpaceUtils.discretizeR(contR, dt.value)

        initP = if(StateSpaceUtils.isStabilizable(discA.transpose(), C.transpose())) {
            Matrix(Drake.discreteAlgebraicRiccatiEquation(discA.transpose(), C.transpose(), discQ, discR))
        } else {
            zeros(states, states)
        }

        P = initP
    }

    fun reset() {
        xHat = zeros(states)
        P = initP
    }

    fun predict(u: Vector<Inputs>, dt: SIUnit<Second>) {
        predict(u, dt, f)
    }

    fun predict(u: Vector<Inputs>, dt: SIUnit<Second>, f: StateFunction<States, Inputs>) {
        val contA = numericalJacobianX(states, states, f, xHat, u)

        val discPair = StateSpaceUtils.discretizeAQTaylor(contA, contQ, dt.value)
        val discA = discPair.first
        val discQ = discPair.second

        if(useRungeKutta) {
            xHat = rungeKutta(f, xHat, u, dt)
        } else {
            xHat = f(xHat, u)
        }

        P = discA * P * discA.transpose() + discQ
        discR = StateSpaceUtils.discretizeR(contR, dt.value)
    }

    fun correct(u: Vector<Inputs>, y: Vector<Outputs>) {
        correct(outputs, u, y, h, discR)
    }

    fun <Rows: Num> correct(rows: Nat<Rows>, u: Vector<Inputs>, y: Vector<Rows>, h: OutputFunction<States, Inputs, Rows>, R: Matrix<Rows, Rows>) {
        val C = numericalJacobianX(rows, states, h, xHat, u)

        val S = C * P * C.transpose() + R
        val K = Matrix<States, Rows>(StateSpaceUtils.lltDecompose(S.transpose().storage).solve((C * P.transpose()).storage).transpose())
        xHat += K * (y - h(xHat, u))
        P = (eye(states) - K * C) * P
    }
}