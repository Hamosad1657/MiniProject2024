package frc.robot.subsystems.hood

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d

object HoodConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
	val ANGLE_TOLERANCE = Rotation2d()

	/**
	 * For every 144 rotations the encoder sees, the hood moves 16 rotations.
	 *
	 * Multiply by this value to get encoder rotations from hood rotations.
	 */
	const val GEAR_RATIO_ENCODER_TO_HOOD = 16.0 / 144.0

	/**
	 * Lowest hood angle
	 */
	val MIN_ANGLE = 0.0.degrees

	/**
	 * Highest hood angle
	 */
	val MAX_ANGLE = 180.0.degrees
}
