package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.Num
import frc.team4069.saturn.lib.mathematics.matrix.Vector
import frc.team4069.saturn.lib.mathematics.matrix.set
import frc.team4069.saturn.lib.mathematics.matrix.zeros
import kotlin.jvm.functions.FunctionN

fun <Rows: Num, Cols: Num, States: Num> numericalJacobian(rows: Nat<Rows>, cols: Nat<Cols>, f: (Vector<Cols>) -> Vector<States>, x: Vector<Cols>,
                                                                              epsilon: Double = 1E-5): Matrix<Rows, Cols> {
    val result = zeros(rows, cols)

    for(i in 0 until cols.num) {
        val dxPlus = x.copy()
        val dxMinus = x.copy()
        dxPlus[i, 0] += epsilon
        dxMinus[i, 0] -= epsilon
        val dF = (f(dxPlus) - f(dxMinus)) / (2 * epsilon)

        result.storage.setColumn(i, 0, *dF.storage.ddrm.data)
    }

    return result
}

fun <Rows: Num, States: Num, Inputs: Num, D: Num> numericalJacobianX(rows: Nat<Rows>, states: Nat<States>, f: (Vector<States>, Vector<Inputs>) -> Vector<D>, x: Vector<States>,
                                                            u: Vector<Inputs>): Matrix<Rows, States> {
    return numericalJacobian(rows, states, { x: Vector<States> -> f(x, u) }, x)
}

fun <Rows: Num, States: Num, Inputs: Num> numericalJacobianU(rows: Nat<Rows>, inputs: Nat<Inputs>, f: (Vector<States>, Vector<Inputs>) -> Vector<States>, x: Vector<States>,
                                                             u: Vector<Inputs>): Matrix<Rows, Inputs> {
    return numericalJacobian(rows, inputs, { u: Vector<Inputs> -> f(x, u) }, u)
}