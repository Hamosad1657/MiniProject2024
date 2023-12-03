package com.hamosad1657.lib.mechanisms.shooter

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Length
import kotlin.math.atan
import kotlin.math.sin
import kotlin.math.sqrt
import frc.robot.subsystems.shooter.ShooterConstants.Calculations as Constants

/**
 * @param distanceToBoiler from shooter, get from vision.
 */
@Suppress("LocalVariableName")
fun calculateRequiredAngularVelocity(distanceToBoiler: Length): AngularVelocity {
	val distance = distanceToBoiler.meters
	val Bx: Double = distance / 100.0 + 0.5
	val By: Double = Constants.BOILER_Y
	val Ty: Double = Constants.TURRET_Y
	val My: Double = Constants.MAX_HEIGHT
	val a = (-By - Ty + 2 * My + 2 * sqrt(My * My + By * Ty - By * My - Ty * My)) / (Bx * Bx)
	val b = (By - Ty + a * Bx * Bx) / Bx
	val o = -2 * a * 0 + b
	val angle = atan(o)
	val velocity = sqrt(2 * Constants.G * (My - Ty)) / sin(angle)
	return AngularVelocity.fromMps(velocity, Constants.WHEEL_RADIUS)
}