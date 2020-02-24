package frc.team4069.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import frc.team4069.robot.TestCommand
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.mathematics.units.second

object DrivetrainTests {
    class VerifyEncodersForward : TestCommand.TestPhase<Phase>(Phase.VerifyEncodersForward) {
        var finished = false
        var timer = Timer()

        override fun initialize() {
            timer.reset()
            timer.start()
            Drivetrain.autoReset()
        }

        override fun execute(harness: TestCommand.Harness) {
            Drivetrain.tankDrive(0.2, 0.2)

            if(timer.hasElapsed(1.0)) {
                val leftPos = Drivetrain.leftEncoder.position.value
                val rightPos = Drivetrain.rightEncoder.position.value

                if(leftPos <= 0.0) {
                    harness.reportError("Left encoder should be strictly positive, but is reporting value $leftPos")
                }

                if(rightPos <= 0.0) {
                    harness.reportError("Right encoder should be strictly positive, but is reporting value $rightPos")
                }

                finished = true
            }
        }

        override fun isFinished() = finished
    }


    class VerifyEncodersBackwards : TestCommand.TestPhase<Phase>(Phase.VerifyEncodersBackwards) {
        var finished = false
        var timer = Timer()

        override fun initialize() {
            timer.reset()
            timer.start()
            Drivetrain.autoReset()
        }

        override fun execute(harness: TestCommand.Harness) {
            Drivetrain.tankDrive(-0.2, -0.2)

            if(timer.hasElapsed(1.0)) {
                val leftPos = Drivetrain.leftEncoder.position.value
                val rightPos = Drivetrain.rightEncoder.position.value

                if(leftPos >= 0.0) {
                    harness.reportError("Left encoder should be strictly negative, but is reporting value $leftPos")
                }

                if(rightPos >= 0.0) {
                    harness.reportError("Right encoder should be strictly negative, but is reporting value $rightPos")
                }

                finished = true
            }
        }

        override fun isFinished() = finished
    }

    class VerifyEncodersTurning : TestCommand.TestPhase<Phase>(Phase.VerifyEncodersTurning) {
        var finished = false
        var timer = Timer()

        override fun initialize() {
            timer.reset()
            timer.start()
            Drivetrain.autoReset()
        }

        override fun execute(harness: TestCommand.Harness) {
            Drivetrain.tankDrive(-0.2, 0.2)

            if(timer.hasElapsed(1.0)) {
                val leftPos = Drivetrain.leftEncoder.position.value
                val rightPos = Drivetrain.rightEncoder.position.value

                if(leftPos >= 0.0) {
                    harness.reportError("Left encoder should be strictly negative, but is reporting value $leftPos")
                }

                if(rightPos <= 0.0) {
                    harness.reportError("Right encoder should be strictly positive, but is reporting value $rightPos")
                }

                finished = true
            }
        }

        override fun isFinished() = finished
    }

    enum class Phase {
        VerifyEncodersForward,
        VerifyEncodersBackwards,
        VerifyEncodersTurning
    }
}

