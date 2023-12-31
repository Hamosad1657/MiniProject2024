package frc.robot.subsystems.swerve

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.auto.PIDConstants

object SwerveConstants {
	const val SWERVE_CONFIG_DIR = "swerve"

	val PATH_TRANSLATION_CONSTANTS = PIDConstants(3.0, 0.0, 0.0)
	val PATH_ROTATION_CONSTANTS = PIDConstants(0.0, 0.0, 0.0)

	private const val PATH_MAX_VELOCITY = 1.0
	private const val PATH_MAX_ACCELERATION = 4.0
	val PATH_CONSTRAINTS = PathConstraints(PATH_MAX_VELOCITY, PATH_MAX_ACCELERATION)
}