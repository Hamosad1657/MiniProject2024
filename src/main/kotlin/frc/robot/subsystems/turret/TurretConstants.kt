package frc.robot.subsystems.turret

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees

object TurretConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)

	const val TOLERANCE_DEGREES = 0.0

	/**
	 * Minimum angle of the turret. This may be different from
	 * what is measured by the CANCoder, due to the gear ratio.
	 */
	val MIN_ANGLE = 0.0.degrees
	/**
	 * Maximum angle of the turret. This may be different from
	 * what is measured by the CANCoder, due to the gear ratio.
	 */
	val MAX_ANGLE = 350.0.degrees // TODO: Change to precise value

	const val SCAN_FOR_TAGS_ROTATION_OUTPUT = 0.0

	const val CAMERA_WIDTH = 600

	/**
	 * When multiplying this ratio by the degrees measured by the encoder,
	 * it should return the turret's angle in 0 to 360.
	 */
	const val GEAR_RATIO_ENCODER_TO_TURRET = 0.0

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