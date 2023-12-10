package frc.robot.mechanisms

import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.BOILER_Y
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.G
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.MAX_HEIGHT
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.TURRET_HEIGHT
import frc.robot.subsystems.shooter.ShooterConstants.Calculations.WHEEL_RADIUS
import kotlin.math.*

// Class that contains shooter velocity and hood angle
// and the calculation function will return and instance of this class
// (aka will return both velocity and angle)

val BOILER_LOCATION = Translation2d()

data class HoodShooterState(val hoodAngle: Rotation2d, val angularVelocity: AngularVelocity) {


	companion object {
//		TODO(Hagar and Shaked) if possible make shooter velocity change minimal
		/**
		 * @param distanceToBoiler from shooter, get from vision.
		 */
		fun fromLength(distanceToBoiler: Length): HoodShooterState {
			val distance = distanceToBoiler.meters
			val a = (-BOILER_Y - TURRET_HEIGHT + 2.0 * MAX_HEIGHT + 2.0 *
				sqrt(MAX_HEIGHT.pow(2.0) + BOILER_Y * TURRET_HEIGHT - BOILER_Y * MAX_HEIGHT - TURRET_HEIGHT * MAX_HEIGHT)) /
				distance.pow(2.0)
			val b = (BOILER_Y - TURRET_HEIGHT + a * (distance.pow(2.0))) / (distance)

			val hoodAngle = atan(b).radians
			val shooterVelocityMps = sqrt(2 * G * (MAX_HEIGHT - TURRET_HEIGHT)) / sin(hoodAngle.radians)

			return HoodShooterState(hoodAngle, AngularVelocity.fromMps(shooterVelocityMps, WHEEL_RADIUS))
		}
	}
}