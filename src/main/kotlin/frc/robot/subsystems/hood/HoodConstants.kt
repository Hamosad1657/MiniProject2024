package frc.robot.subsystems.hood

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d

object HoodConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
	val ANGLE_TOLERANCE = Rotation2d()

	const val MIN_ENCODER_POSITION = 0.0
	const val MAX_ENCODER_POSITION = 359.0
	const val TOP_LIMIT_ENCODER_POSITION = 330.0

	/**
	 * Lowest hood angle
	 */
	val MIN_ANGLE = 0.degrees

	/**
	 * Highest hood angle
	 */
	val MAX_ANGLE = 55.degrees // 361.2 is encoder position at highest

	/**
	 * For every 144 rotations the encoder sees, the hood moves 16 rotations.
	 *
	 * Multiply by this value to get encoder rotations from hood rotations.
	 */
	val GEAR_RATIO_ENCODER_TO_HOOD = (MAX_ANGLE - MIN_ANGLE).degrees / (MAX_ENCODER_POSITION - MIN_ENCODER_POSITION)

}
