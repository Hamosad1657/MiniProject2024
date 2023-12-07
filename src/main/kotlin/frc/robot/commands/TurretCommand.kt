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
			TurretSubsystem.setAngleSetpoint(angle.degrees)
		} andThen waitUntil {
			val error = angle - currentAngleDeg
			abs(error) <= TurretConstants.TOLERANCE_DEGREES
		} andThen TurretSubsystem.runOnce {
			motor.stopMotor()
		}
	}
}

fun TurretSubsystem.fullTurnCommand(): Command =
	withName("fullTurn") {
		getToAngleCommand(farthestTurnAngle.degrees)
	}

/**
 * Slowly turn to one direction until limit is pressed,
 * slowly turn to the other direction until limit is pressed
 * end if saw tag of specified ID for more than [Constants.TAG_DETECTION_TIME_SEC]
 * or if made two turns and didn't see the tag
 */
fun TurretSubsystem.searchForTagCommand(tagID: Int, trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
	return TurretSubsystem.fullTurnCommand() andThen TurretSubsystem.fullTurnCommand() until {
		seeingTag(
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
					if (lastTargetLeftCornerX != null) guessTurnAngleByTargetCorner(
						lastTargetLeftCornerX!!
					).degrees
					else farthestTurnAngle.degrees
			}

			setAngleSetpoint(currentTurnAngle!!)
		} else {
			lastTargetLeftCornerX = target.detectedCorners.minBy { it.x }.x
			currentTurnAngle = null

			val desiredAngle = currentAngleDeg - target.yaw
			setAngleSetpoint(desiredAngle.degrees)
		}
	}.finallyDo {
		motor.stopMotor()
	}
}

fun TurretSubsystem.closedLoopTeleopCommand(
	cwRotationSupplier: () -> Double,
	ccwRotationSupplier: () -> Double
): Command {
	return TurretSubsystem.run {

		val setpoint =
			(currentAngleDeg + cwRotationSupplier() - ccwRotationSupplier()) * TurretConstants.TELEOP_ANGLE_MULTIPLIER
		setAngleSetpoint(setpoint.degrees)
	}
}
