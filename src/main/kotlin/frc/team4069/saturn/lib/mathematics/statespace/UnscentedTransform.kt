package frc.team4069.saturn.lib.mathematics.statespace

import org.ejml.simple.SimpleMatrix

fun unscentedTransform(
    states: Int,
    covDim: Int,
    sigmas: SimpleMatrix,
    Wm: SimpleMatrix,
    Wc: SimpleMatrix,
    noiseCov: SimpleMatrix? = null
): Pair<SimpleMatrix, SimpleMatrix> {

    val x = Wm.mult(sigmas)

    val y = SimpleMatrix(2 * states + 1, covDim)
    for(i in 0 until 2 * states + 1) {
        y.insertIntoThis(i, 0, sigmas.extractVector(true, i))
    }

    var P = y.transpose().mult(Wc.diag()).mult(y)

    if(noiseCov != null) {
        P += noiseCov
    }

    return x to P
}