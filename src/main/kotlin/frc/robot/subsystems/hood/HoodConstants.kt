package frc.robot.subsystems.hood

import com.hamosad1657.lib.math.PIDGains

object HoodConstants {
	const val GEAR_RATIO_ENCODER_TO_HOOD = 0.0
	const val MAX_HOOD_ANGLE_DEG = 180.0

	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
}
