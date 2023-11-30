package frc.robot.subsystems.turret

object TurretConstants {
	const val kP = 0.0
	const val kI = 0.0
	const val kD = 0.0

	const val TOLERANCE_DEGREES = 0.0

	const val MIN_ANGLE_DEG = 0.0
	const val MAX_ANGLE_DEG = 350.0

	/**
	 * In pixels
	 */
	const val CAMERA_MID_WIDTH = 300

	/**
	 * When multiplying this ratio by the degrees measured by the encoder,
	 * it should return the turret's angle in 0 to 360.
	 */
	const val GEAR_RATIO_ENCODER_TO_TURRET = 0.0
	const val TELEOP_ANGLE_MULTIPLIER = 0.0
}