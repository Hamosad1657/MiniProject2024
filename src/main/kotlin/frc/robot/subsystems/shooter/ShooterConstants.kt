package frc.robot.subsystems.shooter

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.centimeters

object ShooterConstants {
	val PID_GAINS = PIDGains(0.0, 0.0, 0.0)
	val VELOCITY_TOLERANCE = AngularVelocity.fromFalconTicksPer100ms(0.0)

	/**
	 * For every rotation of the motor, the flywheel does 0.9 rotations.
	 * For every rotation of the flywheel, X balls are shot.
	 * Multiply by this value to get motor rotations from balls.
	 *
	 * TODO: Find X by filming the shooter in slow-motion.
	 */
	const val SHOOTER_BALLS_PER_ROTATION = 0.0

	val MAX_VELOCITY = AngularVelocity.fromRpm(6300.0) // Falcon free speed is 6380 according to Vex, I rounded it down

	// TODO Find the correct values
	object Calculations {
		const val BOILER_Y = 2.59
		const val TURRET_HEIGHT = 0.711
		const val MAX_HEIGHT = 3.0
		val WHEEL_RADIUS = 20.centimeters
		const val G = 9.81
	}
}