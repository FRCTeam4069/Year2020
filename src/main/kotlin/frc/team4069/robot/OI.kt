package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.robot.commands.AutoSetFlywheelReferenceCommand
import frc.team4069.robot.commands.CenterToTapeCommand
import frc.team4069.robot.commands.elevator.AutoIndexBallsCommand
import frc.team4069.robot.subsystems.*
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.hid.*
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.mathematics.units.rpm
import frc.team4069.saturn.lib.mathematics.units.velocity
import frc.team4069.saturn.lib.util.deadband
import frc.team4069.saturn.lib.vision.LimelightCamera

object OI {
    val controller = xboxController(0) {

        button(kBumperRight) {
            val command = CenterToTapeCommand()
            var oldValue = LimelightCamera.LEDState.ForceOff
            changeOn {
                command.schedule()
                oldValue = Vision.ledState
                Vision.ledState = LimelightCamera.LEDState.ForceOn
            }
            changeOff {
                command.cancel()
                Vision.ledState = oldValue
            }
        }

        button(kBumperLeft) {
            changeOn {
                Climber.engageBrake()
            }
        }

        button(kBack) {
            changeOn {
                Climber.disengageBrake()
            }
        }

        button(kA) {
            changeOn {
                Drivetrain.gear = Drivetrain.Gear.Low
            }

            changeOff {
                Drivetrain.gear = Drivetrain.Gear.High
            }
        }

        button(kStart) {
            changeOn {
                Drivetrain.gear = Drivetrain.Gear.Low
            }
        }

        button(kX) {
            changeOn {
                TowerOfDoom.setTowerDutyCycle(0.65)
            }

            changeOff {
                TowerOfDoom.setTowerDutyCycle(0.0)
            }
        }

        button(kB) {
            changeOn {
                TowerOfDoom.setTowerDutyCycle(-0.4)
            }

            changeOff {
                TowerOfDoom.setTowerDutyCycle(0.0)
            }
        }
    }

    val operatorController = xboxController(1) {

        pov(POVSide.DOWN) {
            changeOn {
                    Flywheel.enable()
                    Vision.ledState = LimelightCamera.LEDState.ForceOn
                    Flywheel.setReference(1700.rpm)
            }

            changeOff {
                Flywheel.disable()
                Vision.ledState = LimelightCamera.LEDState.ForceOff
            }
        }

        pov(POVSide.UP) {
            changeOn {
                Flywheel.enable()
                Vision.ledState = LimelightCamera.LEDState.ForceOn
                Flywheel.setReference(Flywheel.TRENCH_SHOT_PRESET)
            }

            changeOff {
                Flywheel.disable()
                Vision.ledState = LimelightCamera.LEDState.ForceOff
            }
        }


        button(kX) {
            changeOn(AutoIndexBallsCommand())
        }

        pov(POVSide.LEFT) {
            val command = AutoSetFlywheelReferenceCommand()
            changeOn {
                command.schedule()
            }

            changeOff {
                command.cancel()
            }

        }
        button(kBumperRight) {
            var extended = false
            changeOn {
                if(!extended) {
                    Intake.setPivotState(Intake.PivotPosition.Extended)
                    extended = true
                } else {
                    Intake.setPivotState(Intake.PivotPosition.Retracted)
                    extended = false
                }
            }
        }
    }

    val driveSpeed: Double
        get() = (controller.getTriggerAxis(GenericHID.Hand.kRight) - controller.getTriggerAxis(GenericHID.Hand.kLeft))

    val driveTurn: Double
        get() = controller.getX(GenericHID.Hand.kLeft).deadband(0.2)

    val climberSpeed: Double
        get() = controller.getY(GenericHID.Hand.kRight).deadband(0.2)

    val sliderSpeed: Double
        get() = controller.getX(GenericHID.Hand.kRight).deadband(0.15)

    val intakeSpeed: Double
        get() = (operatorController.getTriggerAxis(GenericHID.Hand.kRight) - operatorController.getTriggerAxis(
            GenericHID.Hand.kLeft
        )).deadband(0.1)

    val towerSpeed: Double
        get() = -operatorController.getY(GenericHID.Hand.kRight).deadband(0.15)

    val colourWheelSpeed: Double
        get() = operatorController.getX(GenericHID.Hand.kLeft).deadband(0.15)
}
