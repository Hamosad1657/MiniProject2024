package frc.robot.subsystems.intake

import edu.wpi.first.hal.HAL
import org.junit.jupiter.api.*
import org.junit.jupiter.api.Assertions.*

/**
 * Example test just to learn how this works.
 */

class IntakeTests {
	val DELTA = 1e-2 // Copied from WPILib's example

	@BeforeEach
	fun setup() {
		HAL.initialize(500, 0)
	}

	@AfterEach
	fun cleanup() {
		IntakeSubsystem.close()
	}

	@Test
	fun collects() {
		IntakeSubsystem.collect()
		assertTrue(IntakeSubsystem.running)
		assertEquals(IntakeConstants.MOTOR_OUTPUT, IntakeSubsystem.currentOutput, DELTA)
	}

	@Test
	fun stops() {
		IntakeSubsystem.stop()
		assertFalse(IntakeSubsystem.running)
		assertEquals(0.0, IntakeSubsystem.currentOutput, DELTA)
	}
}