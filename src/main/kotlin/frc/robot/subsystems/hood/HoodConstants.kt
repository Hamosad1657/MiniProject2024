package frc.robot.subsystems.hood

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d

object HoodConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
	val ANGLE_TOLERANCE = Rotation2d()

	const val GEAR_RATIO_ENCODER_TO_HOOD = 0.0

	/**
	 * Lowest hood angle
	 */
	val MIN_HOOD_ANGLE = 0.0.degrees

	/**
	 * Highest hood angle
	 */
	val MAX_HOOD_ANGLE = 180.0.degrees
}
