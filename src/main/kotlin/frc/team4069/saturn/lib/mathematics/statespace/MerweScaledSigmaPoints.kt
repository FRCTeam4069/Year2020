package frc.team4069.saturn.lib.mathematics.statespace

import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.Num
import frc.team4069.saturn.lib.mathematics.matrix.Vector
import frc.team4069.saturn.lib.mathematics.matrix.zeros
import org.ejml.dense.row.CommonOps_DDRM
import org.ejml.dense.row.factory.DecompositionFactory_DDRM
import org.ejml.simple.SimpleMatrix
import kotlin.math.pow


class MerweScaledSigmaPoints<States : Num>(
    val states: Nat<States>,
    val stateCov: Vector<States>,
    val alpha: Double,
    val kappa: Double,
    beta: Double
) {
    var Wc = SimpleMatrix(1, 2 * states.num + 1)
    var Wm = SimpleMatrix(1, 2 * states.num + 1)

    val numSigmas get() = 2 * states.num + 1

    init {
        computeWeights(beta)
    }

    /**
     * Calculate the sigma points
     *
     * @param x the mean of each state
     */
    fun sigmaPoints(x: Vector<States>, P: Matrix<States, States>): SimpleMatrix {

        val sigmaPoints = SimpleMatrix(2 * states.num + 1, states.num)

        sigmaPoints.concatRows(x.storage)
        val lambda = alpha.pow(2) * (states.num + kappa) - states.num

        val U = if (P.elementSum() == 0.0) {
            Matrix(SimpleMatrix(P.numRows, P.numCols))
        } else Matrix<States, States>(lltDecompose((P * (lambda + states.num)).storage))

        for (i in 0 until states.num) {
            val Ui = Vector<States>(U.storage.extractVector(true, i).transpose())
            sigmaPoints.insertIntoThis(i + 1, 0, (x + Ui).transpose().storage)
            sigmaPoints.insertIntoThis(states.num + i + 1, 0, (x - Ui).transpose().storage)

        }
        return sigmaPoints
    }

    fun computeWeights(beta: Double) {
        val lambda = alpha.pow(2) * (states.num + kappa) - states.num

        val c = .5 / (states.num + lambda)
        Wc = SimpleMatrix(1, 2 * states.num + 1)
        CommonOps_DDRM.fill(Wc.ddrm, c)
        Wc[0, 0] = lambda / (states.num + lambda) + (1 - alpha.pow(2) + beta)
        Wm = SimpleMatrix(1, 2 * states.num + 1)
        CommonOps_DDRM.fill(Wm.ddrm, c)
        Wm[0, 0] = lambda / (states.num + lambda)
    }
}

fun lltDecompose(src: SimpleMatrix): SimpleMatrix {
    val temp = src.copy()
    val chol = DecompositionFactory_DDRM.chol(temp.numRows(), false)
    if (!chol.decompose(temp.getMatrix())) throw RuntimeException("Cholesky failed!")

    return SimpleMatrix.wrap(chol.getT(null))

}