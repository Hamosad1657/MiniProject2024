package frc.robot.subsystems

import com.hamosad1657.lib.units.Length
import frc.robot.mechanisms.HoodShooterState
import frc.robot.subsystems.shooter.ShooterConstants.Calculations
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

const val DELTA = 0.0001

class ShooterTests {
	@Test
	fun shootingSpeedTest() {
		HoodShooterState.fromDistance(Length.fromMeters(4.0)).let {
			assertEquals(1.0202, it.hoodAngle.radians, DELTA)
			assertEquals(7.8636, it.angularVelocity.toMps(Calculations.WHEEL_RADIUS), DELTA)
		}
	}
}