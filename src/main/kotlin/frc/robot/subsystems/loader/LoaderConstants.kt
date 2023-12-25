package frc.robot.subsystems.loader

import com.hamosad1657.lib.math.PIDGains

object LoaderConstants {
	val CONVEYOR_PID_GAINS = PIDGains(0.0, 0.0, 0.0)
	val MAGAZINE_PID_GAINS = PIDGains(0.0, 0.0, 0.0)

	/**
	 * 2.536 balls fit in the conveyor.
	 * The conveyor does a full rotation for every 75 rotations of the motor.
	 * Divided by 2 because half a rotation of the conveyor gets a ball to the other side.
	 *
	 * Multiply by this value to get motor rotations from conveyor balls.
	 */
	const val CONVEYOR_BALLS_PER_MOTOR_ROTATION = 2.536 / 75 / 2

	/**
	 * Loader has slots for 9 balls in a full circle.
	 * The loader does a full rotation for every 125 rotations of the motor.
	 *
	 * Multiply by this value to get motor rotations from loader balls.
	 */
	const val MAGAZINE_BALLS_PER_MOTOR_ROTATION = 9.0 / 125.0
}