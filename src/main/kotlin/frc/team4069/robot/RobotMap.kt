package frc.team4069.robot

object RobotMap {
    object Flywheel {
        const val MASTER_TALON_ID = 10
        const val SLAVE_TALON_ID = 11

        const val ENCODER_A = 0
        const val ENCODER_B = 1
    }

    object Tower {
        const val TALON_ID = 12
        const val INDEXER_ID = 13
    }

    object Hood {
        const val TALON_ID = 9
    }

    object Climber {
        const val SPARK1_ID = 30
        const val SPARK2_ID = 31

    }

    object Intake {
        const val TALON_ID = 40
        const val PIVOT_TALON_ID = 41
    }
    
    object Drivetrain {
        const val LEFT_MASTER = 1
        const val LEFT_SLAVE = 2

        const val SHIFTER_FWD = 0
        const val SHIFTER_BCK = 7

        const val RIGHT_MASTER = 5
        const val RIGHT_SLAVE = 6

        const val RIGHT_ENCODER_A = 4
        const val RIGHT_ENCODER_B = 5

        const val LEFT_ENCODER_A = 6
        const val LEFT_ENCODER_B = 7
    }
}