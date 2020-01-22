package frc.team4069.robot.subsystems.flywheel

import edu.wpi.first.wpilibj.LinearFilter
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.Timer
import frc.team4069.keigen.*
import frc.team4069.robot.map
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceController
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceObserver
import frc.team4069.saturn.lib.mathematics.statespace.StateSpacePlant
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceControllerCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceObserverCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpacePlantCoeffs
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.mathematics.units.derived.Velocity
import frc.team4069.saturn.lib.mathematics.units.derived.Volt
import kotlin.math.absoluteValue

class FlywheelController {
    val plant = StateSpacePlant(FlywheelCoeffs.plantCoeffs)
    val controller = StateSpaceController(FlywheelCoeffs.controllerCoeffs, plant)
    val observer = StateSpaceObserver(FlywheelCoeffs.observerCoeffs, plant)

    val tolerance = 20 // rad/s

    var enabled = false

    private var u = zeros(`1`)

    private var ref = zeros(`2`)
    private var y = zeros(`1`)

    val velocity: SIUnit<AngularVelocity>
        get() = observer.xHat[0].radian.velocity

    var measuredVelocity: SIUnit<AngularVelocity>
        get() = y[0].radian.velocity
        set(value) {
            y[0] = value.value
        }

    var reference: SIUnit<AngularVelocity>
        get() = ref[0].radian.velocity
        set(value) {
            ref[0] = value.value
        }

    val voltage: SIUnit<Volt>
        get() = u[0].volt

    fun update(): SIUnit<Volt> {

        observer.correct(u, y)

        controller.update(observer.xHat, ref)

        this.u = controller.u

        observer.predict(u)

        return if (enabled) {
            this.u[0].volt
        } else 0.volt
    }

    fun enable() {
        enabled = true
    }

    fun disable() {
        enabled = false
    }
}

object FlywheelCoeffs {
    val plantCoeffs = StateSpacePlantCoeffs(
        states = `2`,
        inputs = `1`,
        outputs = `1`,
        A = mat(`2`, `2`).fill(0.9675557895260317, 1.3303056835711813, 0.0, 0.0),
        B = mat(`2`, `1`).fill(1.3303056835711813, 0.0),
        C = mat(`1`, `2`).fill(1.0, 0.0),
        D = mat(`1`, `1`).fill(0.0)
    )

    val controllerCoeffs = StateSpaceControllerCoeffs(
        K = mat(`1`, `2`).fill(0.18409390097067566, 1.0),
        Kff = mat(`1`, `2`).fill(0.75170693, 0.0),
        Umin = mat(`1`, `1`).fill(0.0),
        Umax = mat(`1`, `1`).fill(12.0)
    )

    val observerCoeffs = StateSpaceObserverCoeffs(
        K = mat(`2`, `1`).fill(0.795541093137215, 1.6517591872016555e-16)
    )
}