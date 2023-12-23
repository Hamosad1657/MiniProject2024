package frc.robot

object RobotMap {
	const val DRIVER_A_CONTROLLER_PORT = 0
	const val DRIVER_B_CONTROLLER_PORT = 1

	object Turret {
		const val MOTOR_ID = 4
		const val CANCODER_ID = 3
		const val CW_LIMIT_CHANNEL = 1
		const val CCW_LIMIT_CHANNEL = 2
	}

	object Shooter {
		const val MOTOR_ID = 7
	}

	object Hood {
		const val MOTOR_ID = 6
		const val ENCODER_ID = 5
		const val BOTTOM_LIMIT_CHANNEL = 0
		const val TOP_LIMIT_CHANNEL = 0
	}

	object Conveyor {
		const val CONVEYOR_MOTOR_ID = 10
		const val LOADER_MOTOR_ID = 9
	}

	object Intake {
		const val MOTOR_ID = 8
	}
}