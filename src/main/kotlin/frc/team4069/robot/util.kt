package frc.team4069.robot

import frc.team4069.keigen.Num
import frc.team4069.keigen.Vector
import frc.team4069.keigen.get
import frc.team4069.keigen.set

fun <N: Num> Vector<N>.map(func: (Double) -> Double): Vector<N> {
    val vec = this.copy()
    for(i in 0 until this.numRows) {
        vec[i] = func(vec[i])
    }
    return vec
}