package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.turret.TurretConstants
import frc.robot.subsystems.turret.TurretSubsystem
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs

/**
 * @param desiredAngle Can be any value, is not required to be in 0 to 360
 */
fun TurretSubsystem.getToAngleCommand(desiredAngle: Rotation2d): Command {
	val angle = MathUtil.inputModulus(desiredAngle.degrees, 0.0, 360.0)

	return withName("getToAngle") {
		TurretSubsystem.runOnce {
			TurretSubsystem.setAngle(angle.degrees)
		} andThen waitUntil {
			val error = angle - TurretSubsystem.currentAngleDeg
			abs(error) <= TurretConstants.TOLERANCE_DEGREES
		} andThen TurretSubsystem.runOnce {
			TurretSubsystem.motor.stopMotor()
		}
	}
}

fun TurretSubsystem.fullTurnCommand(): Command =
	withName("fullTurn") {
		getToAngleCommand(TurretSubsystem.farthestTurnAngle.degrees)
	}

/**
 * Slowly turn to one direction until limit is pressed,
 * slowly turn to the other direction until limit is pressed
 * end if saw tag of specified ID for more than [Constants.TAG_DETECTION_TIME_SEC]
 * or if made two turns and didn't see the tag
 */
fun TurretSubsystem.searchForTagCommand(tagID: Int, trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
	return TurretSubsystem.fullTurnCommand() andThen TurretSubsystem.fullTurnCommand() until {
		TurretSubsystem.seeingTag(
			tagID,
			trackedTargetSupplier()
		)
	}
}

fun TurretSubsystem.trackTargetCommand(trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
	var currentTurnAngle: Rotation2d? = null
	var lastTargetLeftCornerX: Double? = null

	return TurretSubsystem.run {
		val target = trackedTargetSupplier()
		if (target == null) {
			if (currentTurnAngle == null) {
				currentTurnAngle =
					if (lastTargetLeftCornerX != null) TurretSubsystem.guessTurnAngleByTargetCorner(
						lastTargetLeftCornerX!!
					).degrees
					else TurretSubsystem.farthestTurnAngle.degrees
			}

			TurretSubsystem.setAngle(currentTurnAngle!!)
		} else {
			lastTargetLeftCornerX = target.detectedCorners.minBy { it.x }.x
			currentTurnAngle = null

			val desiredAngle = TurretSubsystem.currentAngleDeg - target.yaw
			TurretSubsystem.setAngle(desiredAngle.degrees)
		}
	}.finallyDo {
		TurretSubsystem.motor.stopMotor()
	}
}

fun TurretSubsystem.closedLoopTeleopCommand(
	cwRotationSupplier: () -> Double,
	ccwRotationSupplier: () -> Double
): Command {
	return TurretSubsystem.run {

		val setpoint =
			(TurretSubsystem.currentAngleDeg + cwRotationSupplier() - ccwRotationSupplier()) * TurretConstants.TELEOP_ANGLE_MULTIPLIER
		TurretSubsystem.setAngle(setpoint.degrees)
	}
}
