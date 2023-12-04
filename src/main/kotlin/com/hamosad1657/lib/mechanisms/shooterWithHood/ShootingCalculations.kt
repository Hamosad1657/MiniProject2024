package com.hamosad1657.lib.mechanisms.shooterWithHood

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.radians
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.BOILER_Y
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.G
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.MAX_HEIGHT
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.TURRET_HEIGHT
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.WHEEL_RADIUS
import kotlin.math.atan
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt


// Class that contains shooter velocity and hood angle
// and the calculation function will return and instance of this class
// (aka will return both velocity and angle)

class HoodShooterState(hoodAngle: Rotation2d, angularVelocity: AngularVelocity) {
	companion object {

		/**
		 * @param distanceToBoiler from shooter, get from vision.
		 */
		fun fromLength(distanceToBoiler: Length): HoodShooterState {

			val distance = distanceToBoiler.meters
			val a =
				(-BOILER_Y - TURRET_HEIGHT + 2.0 * MAX_HEIGHT + 2.0 * sqrt(MAX_HEIGHT.pow(2.0) + BOILER_Y * TURRET_HEIGHT - BOILER_Y * MAX_HEIGHT - TURRET_HEIGHT * MAX_HEIGHT)) / (distance.pow(
					2.0
				))

			val b = (BOILER_Y - TURRET_HEIGHT + a * (distance.pow(2.0))) / (distance)

			val hoodAngle = (atan(b)).radians
			val shooterVelocityMps = sqrt(2 * G * (MAX_HEIGHT - TURRET_HEIGHT)) / sin(hoodAngle.radians)

			return HoodShooterState(hoodAngle, AngularVelocity.fromMps(shooterVelocityMps, WHEEL_RADIUS))
		}
	}
}