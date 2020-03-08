package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.robot.commands.flywheel.AutoSetFlywheelReferenceCommand
import frc.team4069.robot.commands.drive.CenterToTapeCommand
import frc.team4069.robot.commands.elevator.AutoIndexBallsCommand
import frc.team4069.robot.subsystems.*
import frc.team4069.robot.subsystems.flywheel.Flywheel
import frc.team4069.saturn.lib.hid.*
import frc.team4069.saturn.lib.mathematics.units.rpm
import frc.team4069.saturn.lib.util.deadband
import frc.team4069.saturn.lib.vision.LimelightCamera

object OI {
    val controller = xboxController(0) {

        button(kBumperRight) {
            val command = CenterToTapeCommand()
            changeOn {
                command.schedule()
                Vision.ledState = LimelightCamera.LEDState.ForceOn
            }
            changeOff {
                command.cancel()
                Vision.ledState = LimelightCamera.LEDState.ForceOff
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

        button(kA) {
            changeOn {
                    Flywheel.enable()
                    Vision.ledState = LimelightCamera.LEDState.ForceOn
                    Flywheel.setReference(2350.rpm)
            }

            changeOff {
                Flywheel.disable()
                Vision.ledState = LimelightCamera.LEDState.ForceOff
            }
        }

        button(kY) {
            changeOn {
                Flywheel.enable()
                Vision.ledState = LimelightCamera.LEDState.ForceOn
                Flywheel.setReference(2500.rpm)
                Hood.setPosition(0.75)
            }

            changeOff {
                Flywheel.disable()
                Vision.ledState = LimelightCamera.LEDState.ForceOff
                Hood.setPosition(0.0)
            }
        }

        button(kB) {
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
        get() = -operatorController.getY(GenericHID.Hand.kLeft).deadband(0.15)

    val colourWheelSpeed: Double
        get() = operatorController.getX(GenericHID.Hand.kLeft).deadband(0.15)
}
