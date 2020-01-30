package frc.team4069.robot.subsystems.flywheel

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.math.StateSpaceUtils
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpiutil.math.numbers.N1
import frc.team4069.saturn.lib.mathematics.matrix.*
import frc.team4069.saturn.lib.mathematics.model.gearbox
import frc.team4069.saturn.lib.mathematics.model.kMotorFalcon500
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceController
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceObserver
import frc.team4069.saturn.lib.mathematics.statespace.StateSpacePlant
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceControllerCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceObserverCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpacePlantCoeffs
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.AngularVelocity
import frc.team4069.saturn.lib.mathematics.units.derived.Volt
import kotlin.math.pow

class FlywheelController {
    val plant = LinearSystem<N1, N1, N1>(
        `1`,
        `1`,
        `1`,
        FlywheelCoeffs.Ac,
        FlywheelCoeffs.Bc,
        FlywheelCoeffs.Cc,
        FlywheelCoeffs.Dc,
        FlywheelCoeffs.Umin,
        FlywheelCoeffs.Umax
    )

    val controller = LinearQuadraticRegulator<N1, N1, N1>(
        `1`,
        `1`,
        plant,
        vec(`1`).fill(50.0),  // Qelms
        vec(`1`).fill(12.0),  // Relms
        0.01
    )

    val observer = KalmanFilter<N1, N1, N1>(
        `1`,
        `1`,
        `1`,
        plant,
        vec(`1`).fill(1.5),  // Model stddev
        vec(`1`).fill(0.9),   // Measurement stddev
        0.01
    )

    val kTolerance = 20.radian.velocity
    var atGoal = false
        private set

    val enabled get() = controller.isEnabled

    private var u = zeros(`1`)

    private var ref = zeros(`1`)
    private var y = zeros(`1`)

    val velocity: SIUnit<AngularVelocity>
        get() = observer.xhat[0].radian.velocity

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

    fun update(dt: SIUnit<Second>): SIUnit<Volt> {

        observer.correct(u, y)

        controller.update(observer.xhat, ref)

        this.u = plant.clampInput(controller.u)

        observer.predict(u, dt.value)

        atGoal = abs(reference - velocity) < kTolerance

        return this.u[0].volt
    }

    fun enable() {
        controller.enable()
    }

    fun disable() {
        controller.disable()
    }
}

object FlywheelCoeffs {
    val G = 1.0
    val numMotors = 2
    val J = 0.02 // kgm2
    val motor = gearbox(kMotorFalcon500.copy(freeSpeed = kMotorFalcon500.freeSpeed - 800.rpm), numMotors)

    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf theorem 7.3.1
    // All of these matrices are continuous, wpilib classes discretize them
    val Ac = mat(`1`, `1`).fill((-1 * G.pow(2) * motor.Kt.value) / (motor.Kv.value * motor.R.value * J))
    val Bc = mat(`1`, `1`).fill((G * motor.Kt.value) / (motor.R.value * J))
    val Cc = mat(`1`, `1`).fill(1.0)
    val Dc = mat(`1`, `1`).fill(0.0)

    val Umin = vec(`1`).fill(0.0)
    val Umax = vec(`1`).fill(12.0)
}
