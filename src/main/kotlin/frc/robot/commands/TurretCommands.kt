package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.math.wrap0to360
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.mechanisms.BOILER_LOCATION
import frc.robot.subsystems.turret.TurretSubsystem
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs
import frc.robot.subsystems.turret.TurretConstants as Constants

/**
 * @param desiredAngle Can be any value, is not required to be in 0 to 360
 */
fun TurretSubsystem.getToAngleCommand(desiredAngle: Rotation2d): Command {
	return withName("getToAngle") {
		runOnce {
			getToAngle(desiredAngle)
		} andThen waitUntil {
			val error = wrap0to360(desiredAngle.degrees) - currentAngle.degrees
			abs(error) <= Constants.TOLERANCE_DEGREES
		} finallyDo {
			stopTurret()
		}
	}
}

fun TurretSubsystem.aimTurretCommand(robotPositionSupplier: () -> Pose2d): Command {
	return withName("getToAngle") {
		run {
			// TODO: Someone else check this
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
fun TurretSubsystem.searchForTagCommand(tagID: Int, trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
	return withName("searchForTagCommand") {
		(fullTurnCommand() andThen fullTurnCommand()) until {
			isTagDetected(tagID, trackedTargetSupplier())
		}
	}
}

fun TurretSubsystem.searchForAnyTagCommand(trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
	return withName("searchForAnyTagCommand") {
		(fullTurnCommand() andThen fullTurnCommand()) until {
			isAnyTagDetected(trackedTargetSupplier())
		}
	}
}

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
