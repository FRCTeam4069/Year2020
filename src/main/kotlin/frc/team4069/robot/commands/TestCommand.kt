package frc.team4069.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem

class TestCommand(val testingSubsystem: SaturnSubsystem) : SaturnCommand(testingSubsystem) {

    var testFailed = false

    private val harness = Harness()
    val phases = mutableListOf<TestPhase<*>>()

    fun withPhases(vararg phases: TestPhase<*>): TestCommand {
        this.phases += phases

        return this
    }

    override fun initialize() {
        phases[0].initialize()
    }

    override fun execute() {
        val currentPhase = phases.first()
        currentPhase.execute(harness)
        if (currentPhase.isFinished()) {
            if (!testFailed) {
                println("Test Phase ${currentPhase.phase} passed.")
                phases.removeAt(0)
                testingSubsystem.setNeutral()
                if (phases.isNotEmpty()) {
                    phases[0].initialize()
                }
            } else {
                println("Test failed. Aborting.")
            }
        }
    }

    override fun isFinished(): Boolean {
        return phases.isEmpty() || testFailed
    }

    abstract class TestPhase<Phase>(val phase: Phase) {

        open fun initialize() {}

        abstract fun execute(harness: Harness)

        abstract fun isFinished(): Boolean

        fun Harness.reportError(msg: String) = reportError(phase, msg)
    }


    inner class Harness {
        fun <Phase> reportError(phase: Phase, msg: String) {
            testFailed = true
            DriverStation.reportError("Subsystem ${testingSubsystem.name} reported error during $phase: $msg", false)
        }
    }
}