package frc.robot.subsystems.intake

import com.hamosad1657.lib.math.roundWithPrecision
import com.hamosad1657.lib.units.Length
import frc.robot.mechanisms.HoodShooterState
import frc.robot.subsystems.shooter.ShooterConstants.Calculations
import org.junit.jupiter.api.Test

class ShooterTests {
	@Test
	fun shootingSpeedTest() {
		HoodShooterState.fromDistance(Length.fromMeters(4.0)).let {
			assert(it.hoodAngle.radians.roundWithPrecision(4) == 1.0202)
			assert(it.angularVelocity.toMps(Calculations.WHEEL_RADIUS).roundWithPrecision(4) == 7.8636)
		}
	}
}