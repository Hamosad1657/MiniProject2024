package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.PIDConstants

object SwerveConstants {
	const val SWERVE_CONFIG_DIR = "swerve"

	val PATH_TRANSLATION_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)
	val PATH_ROTATION_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)
}