package frc.robot.subsystems.turret

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees

object TurretConstants {
	// -1900.986


	val PID_GAINS = PIDGains(0.09, 0.0, 0.0)

	const val TOLERANCE_DEGREES = 0.0

	/**
	 * For every 151 rotation the encoder sees, the turret does 20 rotations.
	 * Multiply by this value to get encoder rotations from turret rotations.
	 */
//	const val GEAR_RATIO_ENCODER_TO_TURRET = 20.0 / 151.0
	const val GEAR_RATIO_ENCODER_TO_TURRET = (360 - 13) / 1900.986
	val MIN_POSITION = -1900.986

	val MAX_POSITION = 0.0

	/**
	 * Minimum angle of the turret. This may be different from
	 * what is measured by the CANCoder, due to the gear ratio.
	 */
	val MIN_ANGLE = (-347 + 90).degrees

	/**
	 * Maximum angle of the turret. This may be different from
	 * what is measured by the CANCoder, due to the gear ratio.
	 */
	val MAX_ANGLE = (90).degrees

	const val SCAN_FOR_TAGS_ROTATION_OUTPUT = 0.0

	const val CAMERA_WIDTH = 600

	const val TELEOP_ANGLE_MULTIPLIER = 0.0

	const val TAG_DETECTION_TIME_SEC = 0.5
	/**
	 * This value should be based on how long PhotonVision detects false-positives for.
	 *
	 * The robot code has a loop time of 20 milliseconds. So for example, a detection
	 * time of 500 milliseconds means the tag has to be detected for 25 loops in a row
	 * to count.
	 */
}