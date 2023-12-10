package frc.robot.subsystems.shooter

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.centimeters

object ShooterConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
	val VELOCITY_TOLERANCE = AngularVelocity.fromFalconTicksPer100ms(0.0)

	const val SHOOTER_BALLS_PER_ROTATION = 0.0

	object Calculations {
		const val BOILER_Y = 0.0
		const val TURRET_HEIGHT = 0.0
		const val MAX_HEIGHT = 0.0
		val WHEEL_RADIUS = 20.centimeters
		const val G = 9.81
	}
}