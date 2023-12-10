package frc.robot.subsystems.hood

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.degrees

object HoodConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)

	const val GEAR_RATIO_ENCODER_TO_HOOD = 0.0

	val MIN_HOOD_ANGLE = 0.0.degrees
	val MAX_HOOD_ANGLE = 180.0.degrees
}
