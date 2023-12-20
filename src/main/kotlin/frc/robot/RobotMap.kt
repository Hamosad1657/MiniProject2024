package frc.robot

object RobotMap {
	const val DRIVER_A_CONTROLLER_PORT = 0
	const val DRIVER_B_CONTROLLER_PORT = 1

	object Turret {
		const val MOTOR_ID = 0
		const val CANCODER_ID = 0
		const val CW_LIMIT_PORT = 0
		const val CCW_LIMIT_PORT = 0
	}

	object Shooter {
		const val MOTOR_ID = 0
	}

	object Hood {
		const val MOTOR_ID = 0
		const val LIMIT_CHANNEL = 0
		const val ENCODER_ID = 0
	}

	object Conveyor {
		const val CONVEYOR_MOTOR_ID = 0
		const val LOADER_MOTOR_ID = 0
	}

	object Intake {
		const val MOTOR_ID = 0
	}
}