package frc.robot.subsystems.turret

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.turret.TurretConstants.CAMERA_MID_WIDTH
import frc.robot.subsystems.turret.TurretConstants.MAX_ANGLE_DEG
import frc.robot.subsystems.turret.TurretConstants.MIN_ANGLE_DEG
import frc.robot.subsystems.turret.TurretConstants.TAG_DETECTION_TIME_SEC
import frc.robot.subsystems.turret.TurretConstants.TOLERANCE_DEGREES
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs

object TurretSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Turret.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Turret.CANCODER_ID)

	/** CCW positive, according to standard mathematical conventions (and WPILib). */
	private val currentAngleDeg get() = encoder.position * TurretConstants.GEAR_RATIO_ENCODER_TO_TURRET
	private val farthestTurnAngle get() = if (currentAngleDeg >= 180) MIN_ANGLE_DEG else MAX_ANGLE_DEG

	private val tagDetectionDebouncer = Debouncer(TAG_DETECTION_TIME_SEC)

	init {
		encoder.configSensorDirection(false) // TODO: verify Turret CANCoder is CCW positive
		motor.inverted = false // TODO: verify Turret motor is CCW positive

		encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
		motor.forwardLimit = { currentAngleDeg >= MAX_ANGLE_DEG }
		motor.reverseLimit = { currentAngleDeg <= MIN_ANGLE_DEG }

		motor.config_kP(0, TurretConstants.kP)
		motor.config_kI(0, TurretConstants.kI)
		motor.config_kD(0, TurretConstants.kD)
	}

	/**
	 * @param desiredAngle May be any value, is not required to be in 0 to 360
	 */
	private fun getToAngle(desiredAngle: Rotation2d) {
		motor.set(ControlMode.Position, MathUtil.inputModulus(desiredAngle.degrees, 0.0, 360.0))
	}

	/**
	 * @param desiredAngle May be any value, is not required to be in 0 to 360
	 */
	fun getToAngleCommand(desiredAngle: Rotation2d): Command {
		val angle = MathUtil.inputModulus(desiredAngle.degrees, 0.0, 360.0)

		return run {
			getToAngle(angle.degrees)
		}.until {
			val error = angle - currentAngleDeg
			abs(error) <= TOLERANCE_DEGREES
		}.finallyDo {
			motor.stopMotor()
		}
	}

	fun fullTurnCommand(): Command = getToAngleCommand(farthestTurnAngle.degrees)

	fun trackTargetCommand(trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
		var currentTurnAngle: Rotation2d? = null
		var lastTargetLeftCornerX: Double? = null

		return run {
			val target = trackedTargetSupplier()
			if (target == null) {
				if (currentTurnAngle == null) {
					currentTurnAngle =
						if (lastTargetLeftCornerX != null) guessTurnAngleByTargetCorner(lastTargetLeftCornerX!!).degrees
						else farthestTurnAngle.degrees
				}

				getToAngle(currentTurnAngle!!)
			} else {
				lastTargetLeftCornerX = target.detectedCorners.minBy { it.x }.x
				currentTurnAngle = null

				val desiredAngle = currentAngleDeg - target.yaw
				getToAngle(desiredAngle.degrees)
			}
		}.finallyDo {
			motor.stopMotor()
		}
	}

	private fun guessTurnAngleByTargetCorner(cornerX: Double) =
		if (cornerX > CAMERA_MID_WIDTH) MAX_ANGLE_DEG else MIN_ANGLE_DEG

	fun closedLoopTeleopCommand(cwRotationSupplier: () -> Double, ccwRotationSupplier: () -> Double): Command {
		return run {
			val setpoint =
				(currentAngleDeg + cwRotationSupplier() - ccwRotationSupplier()) * TurretConstants.TELEOP_ANGLE_MULTIPLIER
			getToAngle(setpoint.degrees)
		}
	}

	/**
	 * Slowly turn to one direction until limit is pressed,
	 * slowly turn to the other direction until limit is pressed
	 * end if saw tag of specified ID for more than [TurretConstants.TAG_DETECTION_TIME_SEC]
	 * or if made two turns and didn't see the tag
	 */
	fun searchForTagCommand(tagID: Int, trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
		return fullTurnCommand().andThen(fullTurnCommand()).until { seeingTag(tagID, trackedTargetSupplier()) }
	}

	/**
	 * Return if saw tag of specified ID for over [TurretConstants.TAG_DETECTION_TIME_SEC] seconds.
	 */
	private fun seeingTag(tagID: Int, trackedTarget: PhotonTrackedTarget?): Boolean {
		var seeingTheTagNow = false;

		if (trackedTarget != null) {
			if (trackedTarget.fiducialId == tagID) {
				seeingTheTagNow = true;
			}
		}
		return tagDetectionDebouncer.calculate(seeingTheTagNow)
	}
}
