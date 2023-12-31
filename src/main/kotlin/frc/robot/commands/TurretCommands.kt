package frc.robot.commands

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.mechanisms.BOILER_LOCATION
import frc.robot.subsystems.turret.TurretSubsystem
import org.photonvision.targeting.PhotonTrackedTarget
import frc.robot.subsystems.turret.TurretConstants as Constants

/**
 * @param desiredAngle Can be any value, is not required to be in 0 to 360
 */
fun TurretSubsystem.getToAngleCommand(desiredAngle: Rotation2d): Command {
	return withName("getToAngle") {
		run {
			getToAngle(desiredAngle)
		} until {
			withinTolerance()
		} finallyDo {
			stopTurret()
		}
	}
}

fun TurretSubsystem.aimTurretCommand(robotPositionSupplier: () -> Pose2d): Command {
	return withName("getToAngle") {
		run {
			val robotToBoilerTranslation = robotPositionSupplier().translation.minus(BOILER_LOCATION)
			val robotToBoilerAngle = robotToBoilerTranslation.angle
			val turretToBoilerAngle = robotToBoilerAngle + currentAngle
			getToAngleCommand(turretToBoilerAngle)
		}
	}
}

fun TurretSubsystem.fullTurnCommand(): Command =
	withName("fullTurn") { getToAngleCommand(farthestTurnAngle) }

/**
 * Slowly turn to one direction until limit is pressed, and then
 * slowly turn to the other direction until limit is pressed
 * end if saw tag of specified ID for more than [Constants.TAG_DETECTION_TIME_SEC]
 * or if made two turns and didn't see the tag
 */
//fun TurretSubsystem.searchForTagCommand(tagID: Int): Command {
//	return withName("searchForTagCommand") {
//		(fullTurnCommand() andThen fullTurnCommand()) until { Vision.getTag(tagID) != null }
//	}
//}

//fun TurretSubsystem.searchForAnyTagCommand(): Command {
//	return withName("searchForAnyTagCommand") {
//		(fullTurnCommand() andThen fullTurnCommand()) until { Vision.isAnyTagDetected }
//	}
//}

fun TurretSubsystem.trackTargetCommand(trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
	var currentTurnAngle: Rotation2d? = null
	var lastTargetLeftCornerX: Double? = null

	return withName("trackTargetCommand") {
		run {
			val target = trackedTargetSupplier()
			if (target == null) {
				if (currentTurnAngle == null) {
					currentTurnAngle =
						lastTargetLeftCornerX?.let { guessTurnAngleFromTargetCorner(it) } ?: farthestTurnAngle
				}
				getToAngle(currentTurnAngle!!)
			} else {
				lastTargetLeftCornerX = target.detectedCorners.minBy { it.x }.x
				currentTurnAngle = null

				val desiredAngle = currentAngle - target.yaw.degrees
				getToAngle(desiredAngle)
			}
		} finallyDo {
			stopTurret()
		}
	}
}

fun TurretSubsystem.closedLoopTeleopCommand(
	cwRotationSupplier: () -> Double,
	ccwRotationSupplier: () -> Double,
): Command {
	return withName("closedLoopTeleopCommand") {
		run {
			val teleopInput = cwRotationSupplier() - ccwRotationSupplier()
			val setpoint = (currentAngle.degrees + teleopInput) * Constants.TELEOP_ANGLE_MULTIPLIER
			getToAngle(setpoint.degrees)
		}
	}
}

fun TurretSubsystem.openLoopTeleopCommand(
	cwRotationSupplier: () -> Double,
	ccwRotationSupplier: () -> Double,
): Command {
	return withName("openLoopTeleopCommand") {
		run {
			setWithLimits(ControlMode.PercentOutput, cwRotationSupplier() - ccwRotationSupplier())
		} finallyDo {
			stopTurret()
		}
	}
}
