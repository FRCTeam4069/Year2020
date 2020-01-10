package frc.team4069.robot.subsystem

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Encoder
import frc.team4069.robot.RobotMap
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.TAU

object Flywheel : SaturnSubsystem() {
    private val talon = TalonSRX(RobotMap.Flywheel.MASTER_TALON_ID)
    private val slave = TalonSRX(RobotMap.Flywheel.SLAVE_TALON_ID)

    private val encoder = Encoder(RobotMap.Flywheel.ENCODER_A, RobotMap.Flywheel.ENCODER_B)

    init {
        slave.follow(talon)

        encoder.distancePerPulse = TAU / 2048.0 // encoder ppr = 2048
    }
}